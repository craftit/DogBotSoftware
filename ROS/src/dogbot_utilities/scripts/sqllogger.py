#!/usr/bin/env python
#
# Listen to ROS topics, and log numeric values to a postgres database for analysis
#
# March 2018, Nic Greenway, React AI Ltd 

import rospy
from std_msgs.msg import String, Float64
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState, Imu

import psycopg2

import sys
import math
import numpy.random as npr
from datetime import datetime

# set global variables for fieldnames and the like for messages of interest
DUMMY_DATA_NAME  ="##testdata"
LOGINFO_TABLE = "log_info"

DATA_TYPES = ["FLOAT","INFO","JOINTCONTROLLERSTATE","JOINTSTATE","IMU"]

FIELDS_JOINTCONTROLLERSTATE = ["set_point","process_value","process_value_dot","error","time_step","command","p","i","d","i_clamp","antiwindup"]
FIELDS_JOINT_STATES = ["position","velocity","effort"]
FIELDS_IMU = ["orient_x","orient_y","orient_z","orient_w","vel_x","vel_y","vel_z","acc_x","acc_y","acc_z"]

# some dogbot-specific items
DOGBOT_JOINTS = []
DOGBOT_JOINT_TABLES = []
DOGBOT_SIM_JOINTS = []
DOGBOT_NAME_LOOKUP = {}

for fb in ["f","b"]:
  for lr in ["l","r"]:
    for jnt in ["knee","pitch","roll"]:
      DOGBOT_JOINT_TABLES.append( "{}{}{}".format(fb, lr, jnt) )

for fb in ["front","back"]:
  for lr in ["left","right"]:
    for jnt in ["knee","pitch","roll"]:
      DOGBOT_JOINTS.append("{}_{}_{}".format(fb,lr,jnt))
      DOGBOT_SIM_JOINTS.append("{}_{}_{}".format(fb,lr,jnt))
      DOGBOT_NAME_LOOKUP["{}_{}_{}_joint".format(fb,lr,jnt)] = DOGBOT_JOINT_TABLES[len(DOGBOT_JOINTS)-1]

# TopicHandler object listens to a single ROS topic
class TopicHandler(object):
    def __init__(self, dataStore, topic, jointnum, ns):
      #print "parsing", topic
      self.initialisedOK = False
      self.dummy = False
      
      self.topicname = topic["name"]
      #later: add 'auto' depending on whether msg has a header
      self.timesource = topic["timesource"] if "timesousensor_msgs/JointStaterce" in topic else "live"
      self.live = (self.timesource=="live")
      self.datatype = topic["datatype"]
      self.processing = topic["processing"] if "processing" in topic else None
      
      self.sources = []      
      self.tablename = formatDogbotName( topic["table"], jointnum )
      
      if jointnum != -1:
        self.source = DOGBOT_JOINT_TABLES[jointnum]
      
      #and grab a reference to the tabledata object to load the data into the DB
      self.table = dataStore.getTable(self.tablename, self.datatype)
      
      #now subscribe to the topic
      if self.topicname == DUMMY_DATA_NAME:
        self.dummy = True
        
      else:
        #an actual topic, subscribe
        tname = ns + formatDogbotName( self.topicname, jointnum )
        
        if self.datatype.upper()[0:5]=="FLOAT":
          rospy.Subscriber(tname, Float64, self.handleFloatMsg)  
        
        elif self.datatype.upper()=="JOINTCONTROLLERSTATE":
          rospy.Subscriber(tname, JointControllerState, self.handlePIDStateMsg)          
        
        elif self.datatype.upper()=="JOINTSTATE":
          rospy.Subscriber(tname, JointState, self.handleStateMsg)          
        
        elif self.datatype.upper()=="IMU":
          rospy.Subscriber(tname, Imu, self.handleImuMsg)          
        
        print "subscribing to", tname
      
      self.initialisedOK = True;
        
        
    def isdummy(self):
      return self.dummy;
    
    def generate(self):
      if not self.dummy: return;
      #create some random data (NB creates in clumps!)...
      for i in range(5):
        v = npr.normal(5,2)
        dt = datetime.now().isoformat()
        self.table.addFloat( (self.source,dt,v) )
    
    def handleFloatMsg(self, msg):
      v = float(msg.data)
      dt = self.getLiveTime(msg)
      self.table.addFloat( (self.source,dt,v) )

    def handlePIDStateMsg(self, msg):
      dt = self.getLiveTime(msg)
      self.table.addOther( (self.source,dt,msg.set_point, msg.process_value, msg.process_value_dot, msg.error, msg.time_step, msg.command, msg.p, msg.i, msg.d, msg.i_clamp, bool(msg.antiwindup)) )

    def handleImuMsg(self, msg):
      dt = self.getLiveTime(msg)
      o = msg.orientation
      v = msg.angular_velocity
      a = msg.linear_acceleration
      
      self.table.addOther( (self.source, dt, o.x, o.y, o.z, o.w, v.x, v.y, v.z, a.x, a.y, a.z) )

    def handleStateMsg(self, msg):
      if not self.sources:
        self.setSources(msg)
      dt = self.getLiveTime(msg)
      p = msg.position
      v = msg.velocity
      e = msg.effort
      for i, sourceid in enumerate(self.sources):
        self.table.addOther( (sourceid, dt, p[i], v[i], e[i]) )
    
    def setSources(self, msg):
      #don't take the order of the joints for granted
      for n in msg.name:
        self.sources.append( DOGBOT_NAME_LOOKUP[n] )
    
    def getLiveTime(self, msg=None):
      if self.live:
        dt = datetime.utcnow().isoformat()
      
      elif msg and "header" in msg:
        #extract from the message
        dt = datetime.fromtimestamp( msg.header.stamp ).isoformat()
        
      else:
        #use ROS clock
        dt = datetime.fromtimestamp( rospy.get_time() ).isoformat()
      return dt;


class TableData(object):
    #aggregate data and store to database
    def __init__(self, table, schema, datatype="float"):
      self.tablename = self.fulltablename = table
      if schema and self.fulltablename[:len(schema)] != schema+".":
        self.fulltablename = schema + "." + self.fulltablename
        
      self.setType(datatype)
      self.data = []
    
    def addFloat(self, data):
      self.data.append(data)

    def addinformation(self, data):
      if len(data[2]) > 1000:
        rospy.loginfo("Truncating comments in logging table, length was {}".format(len(data[2])))
        data[2] = data[2][0:1000]
      self.data.append(data)
    
    def addOther(self, data):
      self.data.append(data)
    
    def log(self, conn):
      if len(self.data)==0: return;
      #insert the data into the table
      cursor = conn.cursor()  
      #print "formatting", self.sql2, self.data
      str_data = ','.join(cursor.mogrify(self.sql2, x) for x in self.data)
      cursor.execute(self.sql + str_data) 
      conn.commit()
      cursor.close()
      self.clear()
      return True;
      
    def clear(self):
      self.data = []
    
    def setType(self, datatype):
      datatype = datatype.upper()
      if datatype[0:4]=="INFO": datatype="INFO"
      if datatype[0:4]=="REAL": datatype="FLOAT"
      
      self.datatype = DATA_TYPES.index(datatype)
      sqlbase = "INSERT INTO {table}(sourceid,logtime,{tablefields}) VALUES"
      sql2base = "(%s,TIMESTAMP WITH TIME ZONE %s,{s})"
      
      if self.datatype == 0:
        self.sql = sqlbase.format(table=self.fulltablename, tablefields="value")
        self.sql2 = sql2base.format(s = "%s")
        
      elif self.datatype == 1:
        self.sql = sqlbase.format(table=self.fulltablename, tablefields="comments")
        self.sql2 = sql2base.format(s = "%s")
        
      elif self.datatype == 2:
        self.sql = sqlbase.format(table=self.fulltablename, tablefields=",".join(FIELDS_JOINTCONTROLLERSTATE))
        self.sql2 = sql2base.format(s = ",".join(["%s"]*len(FIELDS_JOINTCONTROLLERSTATE)))
      
      elif self.datatype == 3:
        self.sql = sqlbase.format(table=self.fulltablename, tablefields=",".join(FIELDS_JOINT_STATES))
        self.sql2 = sql2base.format(s = ",".join(["%s"]*len(FIELDS_JOINT_STATES)))

      elif self.datatype == 4:
        self.sql = sqlbase.format(table=self.fulltablename, tablefields=",".join(FIELDS_IMU))
        self.sql2 = sql2base.format(s = ",".join(["%s"]*len(FIELDS_IMU)))

      #print "datatype", self.datatype
      
    
class DataStore(object):
    def __init__(self, conn, schema):
      self.conn = conn
      self.schema = schema
      self.tables = {}
    
    def getTable(self, tablename, datatype="float"):
      if tablename not in self.tables:
        self.createTableObj(tablename, datatype)
      return self.tables[tablename];
        
    def createTableObj(self, tablename, datatype):
      self.tables[tablename] = TableData(tablename, self.schema, datatype)
    
    def log(self):
      #TODO - stagger the logging across different tables
      for table in self.tables:
        self.tables[table].log(self.conn);
    
    
class sqllogger(object):
    
    def __init__(self, ns, verbose):
      self.initialisedOK = False
      self.verbose = verbose
      self.nodetype = "sqllogger"
      rospy.init_node(self.nodetype, anonymous=True)
      self.ns = "/" + ns + "/" if ns else ""
      self.callerID = rospy.get_caller_id()
      
      self.realNamespace = ns if ns else rospy.get_namespace()
      rospy.loginfo("Startup of {} {} in namespace {}".format(self.nodetype, self.callerID, str(self.realNamespace)))
      
      #connect to the database
      self.dbtype = rospy.get_param("{0}dbtype".format(self.ns), "")
      if self.dbtype[0:8] != "postgres":
        rospy.loginfo("Database type {} not implemented!".format(self.dbtype))
        return;
      
      self.conn = None
      self.conn_string = rospy.get_param("{0}postgres/conn_string".format(self.ns), [])
      self.schema = rospy.get_param("{0}postgres/schema".format(self.ns), "")
      
      if not self.dbconnect(self.conn_string):
        rospy.loginfo("Failed to connect to database {}!".format(self.conn_string))
        return;
      elif verbose:
        rospy.loginfo("Connected to database {}".format(self.conn_string))
        
      self.infotable = TableData(LOGINFO_TABLE, self.schema, "info")
      
      #check there are tables in the schema and note logging start
      if not self.logstart():
        rospy.loginfo("Failed to write to logging tables in schema {} at {}!".format(self.conn_string))
        return;
      
      self.ds = DataStore(self.conn, self.schema)
      
      #set up subscribers
      self.topics = rospy.get_param("{0}topics".format(self.ns), [])
      if len(self.topics)==0:
        rospy.loginfo("No topics to subscribe to!")
      
      self.dummydata = False
      self.topichandlers = []
      
      for topic in self.topics:
        self.parsetopic(topic)
        
      self.initialisedOK = True;    
      return;

    
    def parsetopic(self, topic):
      joints = topic["joints"] if "joints" in topic else -1
      
      if str(joints) == "all" or joints == 12:
        topic["joints"] = [True] * 12;
        
      elif joints == -1 or str(joints) == "n/a":
        topic["joints"] = []
        th = TopicHandler(self.ds, topic, -1, self.ns)
        self.topichandlers.append(th)
        self.dummydata = self.dummydata or th.isdummy()
        
      for i, joint in enumerate( topic["joints"] ):
        if joint:
          th = TopicHandler(self.ds, topic, i, self.ns)
          self.topichandlers.append(th)
          self.dummydata = self.dummydata or th.isdummy()

    
    def generate(self):
      #generating any dummy data?
      if not self.dummydata: return;
      for th in self.topichandlers:
        th.generate()
      
    def log(self):
      #bundle up and commit recent data
      self.ds.log()
        
    def dbconnect(self, conn_string):
      try:
        self.conn = psycopg2.connect(conn_string)
        return True;
      except Exception, e:
        rospy.loginfo("Error connecting to database,{}".format(e))
        return False;
    
    def logstart(self):
      info = "Logging started from ROS node of type {} {} in namespace {}".format(self.nodetype, self.callerID, self.realNamespace)
      
      return self.loginfo(info)
        
        
    def loginfo(self, info):
      try:
        dt = datetime.now().isoformat()
        self.infotable.addinformation( (self.callerID, dt, info) )
        self.infotable.log(self.conn)
        return True;
      
      except Exception, e:
        rospy.loginfo("Error logging information to {} in schema {}".format(LOGINFO_TABLE, self.schema))
        return False;


def formatDogbotName(name, j):
    nm = name.format(simjoint=DOGBOT_SIM_JOINTS[j], 
                    tablejoint=DOGBOT_JOINT_TABLES[j],
                    joint=DOGBOT_JOINTS[j]) if j > -1 else name
    return nm;


def usage():
    print('rosrun dogbot_utilities sqllogger.py dogbot true');
    
def main(argv):
    #namespace will often be blank, mapping will happen in the ROS launch files via a group
    ns = argv[1] if len(argv) > 1 and argv[1][0:2] != "__" else ""
    verbose = argv[2] if len(argv) > 2 and argv[2][0:2] != "__" else ""
    verbose = bool(verbose)
    
    node = sqllogger(ns, verbose)
    if not node.initialisedOK:
      rospy.loginfo("Failed to initialise node, exiting")
      return;
    
    rate = rospy.get_param("log_freq",5) #default 5Hz node operation, i.e. commit in batches every 200ms
    rate = rospy.Rate(rate)
    
    while not rospy.is_shutdown():
        node.generate()
        node.log()
        rate.sleep()
            


if __name__ == '__main__':
    try:
      main(sys.argv)
    except rospy.ROSInterruptException:
      pass
    except Exception, e:
      print "Error", str(e); 
