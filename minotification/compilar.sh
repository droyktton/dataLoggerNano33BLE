gcc -DBLUEZ_VERSION_MAJOR=5 -DBLUEZ_VERSION_MINOR=53 -DGATTLIB_LOG_BACKEND_SYSLOG -DGATTLIB_LOG_LEVEL=3 \
-I/home/ale/Downloads/gattlib-master/include -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include \
-I/home/ale/Downloads/gattlib-master/dbus/../include \
-L/usr/local/lib/ -Wall -g   -std=gnu99 \
 minotification.c /usr/local/lib/libgattlib.so /lib/x86_64-linux-gnu/libglib-2.0.so