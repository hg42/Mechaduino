
rm sketch

#arduino --get-pref

arduino --upload Mechaduino.ino | sed 's/^sketch\///'
