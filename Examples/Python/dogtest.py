#!/usr/bin/env python

# set of basic tests to verify the API is working and the DogBot is available
# and wave its legs in a simple motion

import argparse
import time
import math

try:
  import reactrobotics as rr;
  from reactrobotics import pydogbotapi as pda;
  
except Exception, e:
  print "Failed to import reactrobotics, {}".format(e)
  print "Check you have built the code and run the /Scripts/pythonapi.sh install"

# ======= Tests ====================

def basicConnect(options):
    dog = options["dogname"]
    connection = options["connection"]
    print "making connection of type {} to DogBot {}".format(connection, dog)
    try:
      dbapi = options["dbapi"] = pda.OpenAPI(dog, connection)
      if dbapi:
        time.sleep(3.0) # allow time for the connection to initialise
        return 1;
      else:
        print "Called OpenAPI, but no API object!"
        return 0;
    
    except Exception, e:
      print "Error making connection", e
      return 0;

def checkIsReady(servos, silent):
    ready = True
    for srv in servos:
      name = srv.Name()
      if "front" not in name and "back" not in name:
        #not bothered about this one
        continue;
      
      isready = srv.IsReady()
      ready = ready and isready
      if not silent:
        print "{} servo {} ready".format(name, "is" if isready else "is not")
        if not isready:
          print "Details: ", srv.StatusSummary()
          
    return ready;

def checkControlStateIsReady(servos, silent):
    #temporary routine to check for control state ready only, and 
    # ignore Home status (workaround)
    ready = True
    for srv in servos:
      name = srv.Name()
      if "front" not in name and "back" not in name:
        continue;      
      status = srv.StatusSummary()
      ready = ready and ("Ready" in status)

    return ready;
      
def queryRefresh(servos, silent):
    for srv in servos:
      srv.QueryRefresh()
      

def servoStatus(options, silent=False):
    if "dbapi" not in options or not options["dbapi"]:
      if not basicConnect(options):
        if not silent:
          print "Can't access DogBot status, no connection made"
        return 0;
      
    dbapi = options["dbapi"]
    #spin the servos and check whether all are ready to receive commands
    try:
      servos = dbapi.ListServos()
      ready = checkIsReady(servos, silent)
      if not ready:
        queryRefresh(servos, silent)
        print "called QueryRefresh, sleeping..."
        time.sleep(3.0)
        ready = checkIsReady(servos, silent)
        if not ready:
          print "Warning: basing ready status on control state only!"
          ready = checkControlStateIsReady(servos, silent)
      
      if not silent:
        print "DogBot {} ready to receive commands".format("is" if ready else "is not")
      
      return int(ready);
      
    except Exception, e:
      print "Error accessing servo status", e
      return 0;      


def servoStates(options):
    if "dbapi" not in options or not options["dbapi"]:
      if not basicConnect(options):
        print "Can't access DogBot, no connection made"
        return 0;

    try:
      print "\n"
      print "Dogbot {} state summary".format(options["dogname"])
      
      dbapi = options["dbapi"]
      srvs = dbapi.ListServos()
      print "\n"
      print "Servo status"
      print "Name, status, voltage, drive temp, motor temp"
      for srv in srvs:
        name = srv.Name()
        isready = srv.IsReady()
        voltage = srv.SupplyVoltage()
        print "voltage is", voltage
        tempd = srv.DriveTemperature()
        tempm = srv.MotorTemperature()
        print "{} {} {} V {} C {} C".format(name, "Ready" if isready else "Not Ready", voltage, tempd, tempm)

      jnts = dbapi.ListJoints()
      print "\n"
      print "Joint status"
      print "Name, position, velocity, torque"
      for jnt in jnts:
        name = jnt.Name()
        position = jnt.Position()
        velocity = jnt.Velocity()
        torque = jnt.Torque()
        print "{} {} radians  {} m/s  {} Nm".format(name, position, velocity, torque)
        
      return 1;
      
    except Exception, e:
      print "Error accessing servo status", e
      return 0;      


class DogWagger(object):
    def __init__(self, options, wagtype):
      self.options = options
      self.dbapi = options["dbapi"]
      self.joints = self.dbapi.ListJoints()
      self.wagtype = wagtype
      self.wagtime = options["waggletime"]
      
      self.wagLimits = {"roll" : (-0.2, 0.2),
                        "left_knee" : (-0.9, -2.0),
                        "right_knee" : (0.9, 2.0),
                        "pitch" : (-0.8, 0.8)}
      self.wagPeriod = 2.0 #in s
      
      self.trajectoryUpdatePeriod = 0.02
      self.trajectorySet = False
      self.torqueLimit = options["torquelimit"]
      
      #align wag limits with joints
      self.jointLimits = []
      self.interested = []
      self.jointNames = []
      for joint in self.joints:
        name = joint.Name()
        self.jointNames.append(name)
        
        interested = "roll" in name or "pitch" in name or "virtual" in name
        self.interested.append(interested)
        
        for wl in self.wagLimits:
          if wl in name:
            self.jointLimits.append(self.wagLimits[wl])
            break;
      
      print "Joint names:", self.jointNames
      print "interested:", self.interested
      print "joint limits:", self.jointLimits
      
    def wag(self):
      usingTrajectory = False
      
      if self.wagtype == "wave":
        print "Waving legs using DemandPosition interface method"
        
      elif self.wagtype == "waggle":
        print "Waving legs using DemandTrajectory interface method"
        usingTrajectory = True
      
      start = time.time()
      lasttime = start - 100000.0
      wp = self.wagPeriod * 1000.0 #in ms, to compare against time.time()
      toFrom = True
      firstTime = True
      
      while True:
        if (time.time()) - start > self.wagtime:
          break;
        
        if not usingTrajectory or firstTime:
          #use DemandPosition
          duration = (time.time()) - lasttime
          if duration >= wp:
            lasttime = time.time()
            self.demandPosition(toFrom)
            toFrom = not toFrom
            firstTime = False
          
        else:
          #just waggle back and forth, smoothly
          self.demandTrajectory(toFrom)
            
    
    def demandPosition(self, toFrom):
      for i, joint in enumerate(self.joints):
        if not self.interested[i]:
          continue;
        position = self.jointLimits[i][int(toFrom)]
        print "moving {} to {}".format(self.jointNames[i], position)
        #joint.DemandPosition(position, self.torqueLimit)
    
    
    def demandTrajectory(self, toFrom):
      #iterate at the expected frequency and chart a smooth path between points
      #based on a sine wave
      if not self.trajectorySet:
        self.setupTrajectory()
      
      posTo = [ x[int(toFrom)] for x in self.jointLimits ]
      posFrom = [ x[int(not toFrom)] for x in self.jointLimits ]
      wp = self.wagPeriod
      steps = int(wp / trajectoryUpdatePeriod)
      if steps < 10:
        print "Warning, movement period seems short compared to trajectory updates!"
      
      wp *= 1000.0
      tup = trajectoryUpdatePeriod*1000.0
      start = time.time()
      lasttime = start - 100000.0
      factor = wp / math.pi
      step = 0
      
      while true:
        if time.time() - lasttime >= tup:
          now = time.time()
          duration = now - start
          if now - lasttime > tup*1.1:
            print "Warning: test code can't keep up with desired frequency, duration is {} ms, supposed to be {}!".format(now-lasttime, tup)
            
          if duration > wp:
            if abs(step - steps) > 2:
              print "Warning ,expected {} steps, actually achieved {} across the motion!".format(steps, step)
            break;
          
          f2 = math.cos( duration / factor )
          for i, joint in enumerate(self.joints):
            if not self.interested[i]:
              continue;
            pFrom = posFrom[i]
            request = pFrom + (posTo[i] - pFrom)*f2
            joint.DemandTrajectory(request)
            
          lasttime = now
          step += 1
    
    
    def setupTrajectory(self):
      for joint in self.joints:
        joint.setupTrajectory(self.trajectoryUpdatePeriod, self.torqueLimit)
      
        
def wagDogJoints(options, wagtype):
    if not servoStatus(options, True):
      print "DogBot not ready to receive commands"
      return 0;

    try:
      dogw = DogWagger(options, wagtype)
      dogw.wag()
      return 1;
      
    except Exception, e:
      print "Error moving joints", e
      return 0;      

    
def dogtestArgs():
    parser = argparse.ArgumentParser(description="Run some basic tests on the API and your DogBot") 
    parser.add_argument("-d","--dogname", help="The DogBot's name", default="")
    parser.add_argument("-t","--test", help="Test to run: connect, status, state, wave, waggle. Default is all", default="all")
    parser.add_argument("-c","--connection", help="Connection: local, USB, none.  Default is local", default="local")
    parser.add_argument("-wt","--waggletime", help="Time to run waggle test, seconds, default 4", default=4, type=int)
    parser.add_argument("-tl","--torquelimit", help="Torque limit for joints, default 3.5Nm", default=3.5, type=int)
     
    return parser;
    dbapi = options["dbapi"]
    
def main():
    parser = dogtestArgs()
    options = vars( parser.parse_args() )    
    dogname = options["dogname"]
    
    if not dogname:
      dogname = rr.utils.defaultDogName()
      if dogname:
        print "No dog specified, using default: {}".format(dogname)
      else:
        print "Warning, no default dog name found, have you installed all necessary React Robotics software?"
    
    
    test = options["test"]
    print "Running tests: {}, for DogBot: {}".format(test, options["dogname"])
    passed = 0
    tests = 0
    
    try:
      if test in ["all", "connect"]:
        tests += 1
        passed += basicConnect(options)
      
      if test in ["all", "status"]:
        tests += 1
        passed += servoStatus(options)     
    
      if test in ["all", "state"]:
        tests += 1
        passed += servoStates(options)     
    
      if test in ["all", "wave"]:
        tests += 1
        passed += wagDogJoints(options, "wave")     
      
      if test in ["all", "waggle"]:
        tests += 1
        passed += wagDogJoints(options, "waggle")     

    except Exception, e:
      print "Error:", e
    finally:  
      print "Passed {} out of {} tests".format(passed, tests)
      
  
  
if __name__ == '__main__':
    try:
      main()
      
    except Exception, e:
      print "Error ", str(e)