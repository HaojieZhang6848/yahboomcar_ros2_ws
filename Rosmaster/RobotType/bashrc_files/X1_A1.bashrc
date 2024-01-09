# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi
export PATH=/usr/local/cuda-10.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
alias python=python3
ip=$(ip addr show eth0 | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*)
if [ -z $ip ]; then
  ip=$(ip addr show wlan0 | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*)
fi
if [ -z $ip ]; then
  ip=$(ip addr show lo | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*)
fi
export ROS_IP=$ip
export ROS_MASTER_URI=http://$ROS_IP:11311
export ROBOT_TYPE=X1   # R2, X1, X3, X3plus, X7
export RPLIDAR_TYPE=a1  # a1, a2, a3, s1, s2, 4ROS, x3
export CAMERA_TYPE=astrapro  #astrapro, astraplus

case "$ROBOT_TYPE" in
X3 )
    echo "workspace link to X3 + ROBOT_TYPE: $ROBOT_TYPE "
    rm ~/yahboomcar_ws
    ln -s ~/ROS/X3/yahboomcar_ws ~/yahboomcar_ws
;;
X3plus )
    echo "workspace link to X3plus + ROBOT_TYPE: $ROBOT_TYPE "
    rm ~/yahboomcar_ws
    ln -s ~/ROS/X3plus/yahboomcar_ws ~/yahboomcar_ws
;;
R2 )
    echo "workspace link to R2 + ROBOT_TYPE: $ROBOT_TYPE "
    rm ~/yahboomcar_ws
    ln -s ~/ROS/R2/yahboomcar_ws ~/yahboomcar_ws
;;
X1 )
    echo "workspace link to X1 + ROBOT_TYPE: $ROBOT_TYPE "
    rm ~/yahboomcar_ws
    ln -s ~/ROS/X1/yahboomcar_ws ~/yahboomcar_ws
;;
* )
    echo "Error! pls recheck your ROBOT_TYPE + $ROBOT_TYPE "
;;
esac
echo "-----------------------"
echo -e "MY_IP: \033[32m$ROS_IP\033[0m"
echo -e "ROS_MASTER_URI: "
echo -e "\033[32m$ROS_MASTER_URI\033[0m"
echo -e "my_robot: \033[32m$ROBOT_TYPE\033[0m | my_lidar: \033[32m$RPLIDAR_TYPE\033[0m | my_camera: \033[32m$CAMERA_TYPE\033[0m"
echo "-----------------------"

export PATH=/usr/local/cuda-10.2/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export CUDA_ROOT=/usr/local/cuda
#export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/jetson/software/ORB_SLAM2/Examples/ROS

source /opt/ros/melodic/setup.bash --extend
source ~/yahboomcar_ws/devel/setup.bash --extend
source ~/software/library_ws/devel/setup.bash --extend
source ~/software/world_canvas/devel/setup.bash --extend
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/jetson/software/ORB_SLAM2/Examples/ROS/ORB_SLAM2
#export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/jetson/software/ORB_SLAM2/Example/ROS/ORB_SLAM2
#export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/jetson/software/ORB_SLAM2/Examples/ROS

