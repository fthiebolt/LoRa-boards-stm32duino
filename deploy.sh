#!/bin/bash
#
# Deployment script for stm32duino variants
#
# Thiebolt F.   dec.18  initial release
#

#
# Global defs
STM32DUINO_REV=${STM32DUINO_REV:-"1.5.0"}
STM32DUINO_DIR=${STM32DUINO_DIR:-"~/.arduino15/packages/STM32/hardware/stm32/${STM32DUINO_REV}"}
eval STM32DUINO_DIR=${STM32DUINO_DIR}

# usage
echo -e "\n############################################################"
echo -e   "#                                                          #"
echo -e   "#         neOCampus stm32duino variant installer           #"
echo -e   "#                                                          #"
echo -e   "# -------------------------------------------------------- #"

# check if target directory exists
[ -d ${STM32DUINO_DIR} ] || { echo -e "###ERROR: unable to find dir '${STM32DUINO_DIR}' ... aborting" >&2; exit 1; }

echo -e   "# Detected stm32duino install dir:                         #"
printf    "%-80s\n" "${STM32DUINO_DIR}"
echo -e   "# -------------------------------------------------------- #"
sleep 1

# parse current variants directory
echo -e   "#         install variants                                 #"
echo -e   "#                                                          #"
_cpt=0
for variant_dir in $(/bin/ls -d *_variant/); do
    _dir=${variant_dir%%_variant/}
    echo -ne "${_dir} --> install as stm32duino variant [y/N]? : "
    read -e -n 1 answer
    [ "X${answer,,}" != "Xy" ] && { echo -e "cancelled!"; continue; }
    # copy variant dir content
    _destdir=${STM32DUINO_DIR}/variants/${_dir}
    mkdir ${_destdir} >& /dev/null
    rsync -av --dry-run ${variant_dir}/* ${_destdir} >& /dev/null
    [ $? -ne 0 ] && { echo -e "\n#WARNING: unable to rsync ${_dir} in ${_destdir} ... continuing!" >&2; sleep 2; continue; }
    rsync -av ${variant_dir}/* ${_destdir}
    [ $? -ne 0 ] && { echo -e "\n### ERROR while rsync of ${_dir} in ${_destdir} ... aborting!" >&2; exit 1; }
    echo -e "\tsuccessfully installed variant '${_dir}' :)"
    (( _cpt++ ))
done
echo -e   "#                                                          #"
echo -e   "# -------------------------------------------------------- #"
[ "X${_cpt}" == "X0" ] && { exit 0; }

# copy local defs files
echo -e   "#         copy local files                                 #"
echo -e   "#                                                          #"
for local_file in $(/bin/ls *local.txt); do
    if [ -f ${STM32DUINO_DIR}/${local_file} ]; then
        echo -ne "overwrite existing ${STM32DUINO_DIR}/${local_file} file [y/N]? :"
        read -e -n 1 answer
        [ "X${answer,,}" != "Xy" ] && { echo -e "cancelled!"; continue; }
    fi
    cp -af ${local_file} ${STM32DUINO_DIR}/ >& /dev/null
    [ $? -ne 0 ] && { echo -e "\n### ERROR while copying '${local_file}' file to '${STM32DUINO_DIR}' directory ... aborting!" >&2; exit 1; }
    echo -e "\tsuccessfully installed local file '${local_file}'"
done
echo -e   "#                                                          #"
echo -e   "# -------------------------------------------------------- #"

# finialize
echo -e "\nYou may restart your arduino toolchain to have your variants taken into account"
echo -e "\thave a nice day :)"

# The end - Jim Morrison / 1943 - 1971

