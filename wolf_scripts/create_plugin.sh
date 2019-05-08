#!/bin/bash
# $1 path to the root of the plugin
# $2 name of the plugin
# $3 files to be moved

#Generate the necessary dirs
# if [ ! -d $1 ];
# then
#     mkdir $1
# fi

# if [ ! -d $1/include/$2 ];
# then
#     # mkdir $1/include
#     mkdir $1/include/$2
# fi
# if [ ! -d $1/src ];
# then
#     mkdir $1/src
# fi
root_dir=$(echo $1 | rev | cut -d '/' -f 2- | rev)
if [ ! -d $root_dir/$2 ];
then
    cp -a ../Skeleton $root_dir
    mv $root_dir/Skeleton $root_dir/$2
    mv $root_dir/$2/include/skeleton $root_dir/$2/include/$2
fi

for folder in capture factor feature landmark processor sensor yaml ; do
    if [ ! -d $1/include/$2/$folder ];
    then
        mkdir $1/include/$2/$folder
    fi
    if [ ! -d $1/src/$folder ];
    then
        mkdir $1/src/$folder
    fi
done
for file in $(cat $3); do
    head=$(echo $file | cut -d '/' -f 1)
    if [ "$head" = "include" ];
    then
        folder=$(echo $file | cut -d '/' -f 3)
        suffix=$(echo $file | cut -d '/' -f 4-)
        line=$(ag "HDRS_"${folder^^} $1/CMakeLists.txt | cut -d ':' -f 1 | head -1)
        line=$(($line + 1))
        echo $line " && " $file " && " $folder " && " $suffix
        if [ "$suffix" = "" ];
        then
            line=$(ag "HDRS_BASE" $1/CMakeLists.txt | cut -d ':' -f 1 | head -1)
            line=$(($line + 1))
            suffix=$folder
            sed  -i ""$line"i $head/$2/$suffix" $1/CMakeLists.txt
            cp $file $1/$head/$2/$suffix
        else
            sed  -i ""$line"i $head/$2/$folder/$suffix" $1/CMakeLists.txt
            cp $file $1/$head/$2/$folder/$suffix
        fi
    elif [ "$head" = "src" ];
    then
        folder=$(echo $file | cut -d '/' -f 2)
        suffix=$(echo $file | cut -d '/' -f 3-)
        # ag "SRCS_"$folder $1/CMakeLists.txt
        line=$(ag "SRCS_"${folder^^} $1/CMakeLists.txt | cut -d ':' -f 1 | head -1)
        line=$(($line + 1))
        echo $line " && " $file " && " $folder " && " $suffix
        if [ "$suffix" = "" ];
        then
            line=$(ag "SRCS_BASE" $1/CMakeLists.txt | cut -d ':' -f 1 | head -1)
            line=$(($line + 1))
            suffix=$folder
            sed  -i ""$line"i $file" $1/CMakeLists.txt
            cp $file $1/$head/$suffix
        else
            sed  -i ""$line"i $file" $1/CMakeLists.txt
            cp $file $1/$head/$folder/$suffix
        fi
    else
        cp $file $1/$file
    fi
done
for f in $(cat $3); do
    hhead=$(echo $f | cut -d '/' -f 1)
    if [ "$hhead" = "include" ];
    then
        ffolder=$(echo $f | cut -d '/' -f 3)
        ssuffix=$(echo $f | cut -d '/' -f 4-)
        inc=$ffolder/$ssuffix
    else
        continue
    fi
    for ff in $(cat $3); do
        head=$(echo $ff | cut -d '/' -f 1)
        if [ "$head" = "include" ];
        then
            folder=$(echo $ff | cut -d '/' -f 3)
            suffix=$(echo $ff | cut -d '/' -f 4-)
            if [ "$suffix" = "" ];
            then
                new_path=$1/$head/$2/$folder
                sed -i -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@g" $new_path
            else
                new_path=$1/$head/$2/$folder/$suffix
                sed -i -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@g" $new_path
            fi
            # sed -n -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@pg" $new_path
            # sed -i -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@g" $new_path
        elif [ "$head" = "src" ];
        then
            folder=$(echo $ff | cut -d '/' -f 2)
            suffix=$(echo $ff | cut -d '/' -f 3-)
            # sed -n -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@pg" $new_path
            if [ "$suffix" = "" ];
            then
                new_path=$1/$head/$folder
                sed -i -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@g" $new_path
            else
                new_path=$1/$head/$folder/$suffix
                sed -i -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@g" $new_path
            fi
        else
            # sed -n -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@pg" $new_path
            sed -i -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@g" $1/$ff
        fi
    done
done