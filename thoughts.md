gps/fix - simulator na gps
my node - sub na lidar, odom a gps/simulator
udp_pub - berie recoFiit packet a vytvara ackermann msg

ackermann_to_vesc - berie ackermann a vytvara vesc command na speed a angle
vesc_to_odom - berie vesc voltage atd. a vytvara odom

topics:

gps - pub ✅ sub ✅
lidar - pub(vesc node v jetson) ✅ sub ✅
odom - pub ✅ sub ✅
ackermann - pub ✅ sub ✅

TODO:
 - overit spravnost topic mien z jetsonu
 - v launch file zmenit hodnoty asi podla vesc configu
 - ak funguje ^ : spravit novy launch file z jetsonu aj z recofiit pre ostatne auta

zapinanie:
 - pred zapnutim zmenit v my_node.cpp a udp_to_pub.cpp lokaciu config filov
 - prekompilovat cez `colcon build`
 - spustit vesc node ktorym sme ovladali auto cez laptop a skusali lidar
 - v druhom terminali spustit `ros2 launch tp5_ros vesc_gps_sim_launch.xml`
