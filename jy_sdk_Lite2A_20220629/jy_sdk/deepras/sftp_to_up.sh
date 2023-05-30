#!/usr/bin/expect
#exit

# set passwd "test123"
set passwd "'"
spawn scp build/jy_exe  ysc@192.168.1.120:/home/ysc/jy_exe/bin/jy_exe
# spawn scp build/deepras  test@192.168.1.120:/home/test/jy_application/deepras/build
# spawn scp build/deepras  test@192.168.1.120:/home/test/DrSDK.lnx/deepras/build
expect {
  "密码："
        {
          send "$passwd\n"
        }
   "pass"
        {
          send "$passwd\n"
        }
   "yes/no"
        {
          sleep 5
          send_user "send yes"
          send "yes\n"
        }
   eof
    {
        sleep 5
        send_user "eof\n"
    }
}
set timeout 3000
send "exit\r"
expect eof
