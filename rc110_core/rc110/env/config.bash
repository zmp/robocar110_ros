config_file=~/.config/rc110/service.conf

if [ ! -f $config_file ]; then
    mkdir -p ~/.config/rc110
    cp $(dirname "$0")/service_template.conf $config_file
fi
source $config_file
