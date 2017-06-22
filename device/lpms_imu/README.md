# README

## Under Linux, non root users receive read and write access to /dev/ttyUSB0    
  Firstly, check the permissions attribute of the /dev/ttyUSB0 and enter it at the terminal:
        ll /dev/ttyUSB0
  Secondly,two method to slove this problem:
        1.sudo chmod 666 /dev/ttyUSB0
          However, this method is one-time, pull out the console line, and then the file will be reset, so it will continue to report errors
        2.sudo usermod -aG dialout username
          Because by default, only root users and users belonging to the dialout group will have read and write privileges, so it's OK to add
          your users directly to the dialout group. When you have finished the command, you need to logout it and it will take effect forever.



