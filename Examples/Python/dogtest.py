#!/usr/bin/env python

# set of basic tests to verify the API is working and the DogBot is available


try:
  import reactrobotics as rr;
  
except Exception, e:
  print "Failed to import reactrobotics, {}".format(e)
  print "Check you have built the code and run the /Scripts/pythonapi.sh install"
  

def main():
  
  
  
  
if __name__ == '__main__':
    try:
      main()
      
    except Exception, e:
      print "Error ", str(e)