from serial import Serial
#sudo chmod 777 /dev/ttyACM0 #to open port
class HiwinGrip(object):
  def __init__(self, port, baudrate = 19200):
    self.serial = Serial(port,baudrate=baudrate, timeout=0.1, write_timeout=0.1)

  def ResetMotion(self):
    cmd = "310\n"
    self.serial.write(cmd.encode())
  def RunMove(self,val):
    if(val>10 and val <=16):
      val = int(val)
      print("val",val)
    elif val > 16:
      val = 16
    cmd = str(int(val*10))+"\n"
    self.serial.write(cmd.encode())
  def RunForce(self, key):
    if (key == "L" or key == "l"): # low force
      cmd = str(170)+"\n"
    elif (key == "M" or key == "m"): # medium force
      cmd = str(180)+"\n"
    elif (key == "H" or key == "h"): # high force
      cmd = str(190)+"\n"
    else:
      cmd = str(180)+"\n"
    self.serial.write(cmd.encode())
