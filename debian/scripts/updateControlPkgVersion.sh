#!/bin/bash

################# Find Script directory #####################
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  TARGET="$(readlink "$SOURCE")"
  if [[ $TARGET == /* ]]; then
    SOURCE="$TARGET"
  else
    DIR="$( dirname "$SOURCE" )"
    SOURCE="$DIR/$TARGET" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
  fi
done
RDIR="$( dirname "$SOURCE" )"
DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"

cd $DIR
echo "Found scribt in directory: '$DIR'"

#################### Get Version number ########################
while IFS= read -r line
do
    extension="${line##*.}"
    base="${line%.*}"
    case $base in
        major ) MAJOR=$extension ;;
        minor ) MINOR=$extension ;;
        patch ) PATCH=$extension ;;
    esac
done < "./version"

echo "Updating using version $MAJOR.$MINOR"

#################### Loop through control file ################
regex_findPkg="^Package:"
regex_findDepend="^Depends:"
regex_start_space="^[[:space:]]"
regex_findVersioned="[0-9\.]$"
regex_exludeVersioned="[[:space:]](liblua5.3-dev|libboost.*-dev)"
isDepend="false" 
I=0
input="../control" #"../control"
while IFS= read -r line ; do
    I=$(expr $I + 1 )
    oldLine=$line
    if [[ $line =~ $regex_findPkg ]] && [[ $line =~ $regex_findVersioned ]] && [[ ! $line =~ $regex_exludeVersioned ]] ; then 
        isDepend="false"     
        while [[ $line =~ $regex_findVersioned ]] && [[ ! $line =~ $regex_exludeVersioned ]] && [[ -n $line ]] ; do
            len=${#line}
            lenmm=$(expr $len - 1 )
            line=${line:0:$lenmm}
            #echo "      $line"
        done
        awk -v NRi="$I" -v vnum="$line$MAJOR.$MINOR" 'NR==NRi {$0=vnum} 1' $input > ./tmp && mv tmp $input
        echo "debian/control:$I: $oldLine -> $line$MAJOR.$MINOR"
    elif [[ $line =~ $regex_findDepend ]] || $( [[ "$line" =~ $regex_start_space ]] && [[ $isDepend == "true" ]] ); then 
        isDepend="true"
        newLine=""
        changed=0
        if [[ $line =~ $regex_start_space ]] ; then 
            newLine=" "
        fi
        for l in $line ; do
            regex=",$"
            commaRemoved=0
            if [[ $l =~ $regex ]] && [[ ! $line =~ $regex_exludeVersioned ]]; then
                l=${l:0:$(expr ${#l} - 1 )}
                commaRemoved=1
            fi
            regex="[1-9]*\.[1-9]*"
            if [[ $l =~ $regex ]] && [[ ! $line =~ $regex_exludeVersioned ]] ; then
                changed=1
                #echo "$l"

                while [[ $l =~ $regex_findVersioned ]] && [[ ! $line =~ $regex_exludeVersioned ]] && [[ -n $l ]] ; do
                    lenmm=$(expr ${#l} - 1 )
                    l=${l:0:$lenmm}
                    #echo "      $l"
                done

                l="$l$MAJOR.$MINOR"
            fi
            if [[ $commaRemoved -eq 1 ]] ; then 
                l="$l, "
            else
                l="$l "
            fi
            newLine="$newLine$l"
        done
        #echo "$newLine"
        if [[ $changed -eq 1 ]] ; then
            awk -v NRi="$I" -v vnum="$newLine" 'NR==NRi {$0=vnum} 1' $input > ./tmp && mv tmp $input
            echo "debian/control:$I: $oldLine -> $newLine"
        fi

    else
        isDepend="false"
    fi
done < "$input"

#################### Update install file version ################

RW_CMAKE="sdurw-cmake$MAJOR.$MINOR.install"
RWS_CMAKE="sdurws-cmake$MAJOR.$MINOR.install"
RWHW_CMAKE="sdurwhw-cmake$MAJOR.$MINOR.install"
RWSIM_CMAKE="sdurwsim-cmake$MAJOR.$MINOR.install"

REG_RW="^sdurw-cmake[0-9]*\.[0-9]*\.install"
REG_RWS="^sdurws-cmake[0-9]*\.[0-9]*\.install"
REG_RWHW="^sdurwhw-cmake[0-9]*\.[0-9]*\.install"
REG_RWSIM="^sdurwsim-cmake[0-9]*\.[0-9]*\.install"

FILES=../*

for f in $FILES ; do 
    if [[ -e $f ]] ; then 
        if [[ $(basename -- $f) =~ $REG_RW ]] && [[ ! $(basename -- $f) == $RW_CMAKE ]] ; then 
            mv $f "../$RW_CMAKE"
            echo "Rename: $(basename -- $f) -> $RW_CMAKE "
            sed -i "s/[0-9]*\.[0-9]*/$MAJOR.$MINOR/" $(dirname -- $f)/$RW_CMAKE
        fi
        if [[ $(basename -- $f) =~ $REG_RWS ]]  && [[ ! $(basename -- $f) == $RWS_CMAKE ]]; then 
            mv $f "../$RWS_CMAKE"
            echo "Rename: $(basename -- $f) -> $RWS_CMAKE "
            sed -i "s/[0-9]*\.[0-9]*/$MAJOR.$MINOR/" $(dirname -- $f)/$RWS_CMAKE
        fi
        if [[ $(basename -- $f) =~ $REG_RWHW ]]  && [[ ! $(basename -- $f) == $RWHW_CMAKE ]] ; then 
            mv $f "../$RWHW_CMAKE"
            echo "Rename: $(basename -- $f) -> $RWHW_CMAKE "
            sed -i "s/[0-9]*\.[0-9]*/$MAJOR.$MINOR/" $(dirname -- $f)/$RWHW_CMAKE
        fi
        if [[ $(basename -- $f) =~ $REG_RWSIM ]]  && [[ ! $(basename -- $f) == $RWSIM_CMAKE ]]; then 
            mv $f "../$RWSIM_CMAKE"
            echo "Rename: $(basename -- $f) -> $RWSIM_CMAKE "
            sed -i "s/[0-9]*\.[0-9]*/$MAJOR.$MINOR/" $(dirname -- $f)/$RWSIM_CMAKE
        fi

    fi

done