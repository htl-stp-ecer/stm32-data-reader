USER=pi
HOST="${RPI_HOST:-10.101.156.14}"
DIR=/home/pi/flashFiles

docker compose up

ssh $USER@$HOST "mkdir -p $DIR" || exit 1
scp -r flashFiles/* $USER@$HOST:$DIR/ || exit 1
scp build/Firmware/wombat.bin $USER@$HOST:$DIR/wombat.bin || exit 1

ssh $USER@$HOST "sudo systemctl stop stm32_data_reader.service"
ssh $USER@$HOST "cd $DIR && bash ./flash_wombat.sh"
ssh $USER@$HOST "sudo systemctl start stm32_data_reader.service"
