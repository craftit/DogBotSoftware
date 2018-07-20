#!/usr/bin/env python

# utility functions to help when using React Robotics software via the Python API 

import os
import json

__CONFIGFOLDER__  = os.path.abspath( os.path.expanduser("~/.config/dogbot") )


def defaultDogName(silent=True):
    try:
      defaultfile = os.path.join(__CONFIGFOLDER__, "robot.json")
      
      with open(defaultfile, "r") as jsondata:
        dogfile = json.load(jsondata)
        if "devices" not in dogfile:
          if not silent:
            print "Found robot.json, but no devices listed"
          return "";
        
        dogname = dogfile["name"] if "name" in dogfile else "unspecified"
        return str(dogname);

    except Exception, e:
      if not silent:
        print "Error accessing configuration from robot.json: ", e
      return "";
      

def checkInstall(silent=True):
  ok = True
  
  dogname = defaultDogName(silent)
  ok = ok and bool(dogname)
  
  # add more checks...
  return ok;
