script_path=$(realpath "$(dirname "${BASH_SOURCE[0]}")")

# Usual ros setup.bash
source ${script_path}/setup.bash

# Namespace and hostname variables.
source ${script_path}/rc_env.bash

# User maintained configuration.
config_file=~/.config/rc110/config.bash
if [ ! -f $config_file ]
then
    mkdir -p ~/.config/rc110
    cp $(rospack find rc110)/config/config_template.bash $config_file
fi
source ${config_file}
