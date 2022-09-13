# script to enable maxN mode for build
eval $(cat /var/lib/nvpmodel/status | tr : =)  # get last mode as "pmode"

if [ 0000 -ne ${pmode} ]            # if it's not MaxN,
then
    echo -e "\033[1;31m\n Using all CPU cores during build. Requires entering sudo!\n\033[0m"

    sudo nvpmodel -m 0              # set MaxN
    function cleanup {
        sudo nvpmodel -m ${pmode}  # on exit from build, revert to the last mode
    }
    trap cleanup EXIT
fi