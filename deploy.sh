USER=pi
HOST=192.168.31.75
DIR=/home/pi/flashFiles
docker compose up
scp build/Firmware/wombat.bin $USER@$HOST:$DIR/wombat.bin
ssh $USER@$HOST "cd $DIR && bash ./wallaby_flash"