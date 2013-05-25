#!/bin/sh

# Name of ach channel to send zmp trajectory over
HUBO_ZMP_CHAN='hubo-zmp-traj'

# Opens the ach channel HUBO_ZMP_CHAN,
# Start the achd daemon to push the zmp trajectory to HUBO
# and starts the zmp-daemon program
Start()
{
    AchOpen
    AchdStart
#    echo 'Starting zmp-daemon'
#    ./build/zmp-daemon myhubo.kinbody.xml &
}

# Closes the ach channel and kills the achd daemon
Stop()
{
    AchClose
    AchdStop
}

# Opens the HUBO_ZMP_CHAN channel with permissions and
# large enough buffer for the zmp trajectory
AchOpen()
{
    echo 'Opening ach channel'
    sudo ach -1 -C $HUBO_ZMP_CHAN -m 3 -n 1000000 -o 666
}

# Closes the HUBO_ZMP_CHAN channel
AchClose()
{
    echo 'Closing ach channel'
    sudo ach -U $HUBO_ZMP_CHAN
}

AchRestart()
{
    AchClose
    AchOpen
}
# Starts the ach daemon, pushing the
# HUBO_ZMP_CHAN over ach to hubo-linux PC
AchdStart()
{
    echo 'Starting achd push'
    sudo achd -r push 10.84.108.135 $HUBO_ZMP_CHAN
}

# Kills the achd daemon process
AchdStop()
{
    echo 'Killing achd'
    sudo killall -9 achd
}

ShowUsage()
{
    echo
    echo 'USAGE:'
    echo
    echo 'start     : create ach channel, start achd, run zmp-daemon'
    echo 'stop      : close ach channel, stop achd'
    echo 'achopen   : open ach channel hubo-zmp-traj'
    echo 'achclose  : close ach channel'
    echo 'achdstart : start the achd daemon'
    echo 'achdstop  : stop the achd daemon'
    echo
}

case "$1" in
    'start')
        Start $@
    ;;

    'stop')
        Stop $2
    ;;
    'achopen')
        AchOpen $2
    ;;

    'achclose')
        AchClose $2
    ;;

    'achrestart')
        AchRestart $2
    ;;

    'achdstart')
        AchdStart $2
    ;;

    'achdstop')
        AchdStop $2
    ;;

    *)
        ShowUsage
        exit 1
    ;;
esac

exit 0
