NAME=
BASE=
TYPE=
while getopts ":t:n:b:" OPTION
do
  case ${OPTION} in
    t)
       TYPE=${OPTARG}
       ;;
    n)
       NAME=${OPTARG}
       ;;
    b)
       BASE=${OPTARG}
       ;;
    *) 
       showHelp 
       exit
       ;;
  esac
done
if [ -z "$TYPE" ] || [ -z "$NAME" ] || [ -z "$BASE" ]
then
   showHelp	
   exit
fi

NAME=$(LowerCase $NAME)
NAME_CAP=$(UpperCase $NAME)
BASE=$(LowerCase $BASE)
BASE_CAP=$(UpperCase $BASE)
TYPE=$(LowerCase $TYPE)
TYPE_CAP=$(UpperCase $TYPE)

if ! [[ $NAME =~ $TYPE ]];
then
    NAME="$TYPE"_"$NAME";
fi
if ! [[ $BASE =~ $TYPE ]];
then
    BASE="$TYPE"_"$BASE";
fi

# Useful derivatives
TYPE_CAP1=$(UpperCaseFirstLetter $TYPE)
CLASSNAME=$(echo ${NAME##*_})
CLASSNAME=Processor$(echo "$(echo "$CLASSNAME" | sed 's/.*/\u&/')") 
BASECLASSNAME=$(echo ${BASE##*_})
BASECLASSNAME=Processor$(echo "$(echo "$BASECLASSNAME" | sed 's/.*/\u&/')") 
