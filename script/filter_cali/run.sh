img_root=/media/psf/Home/Documents/code/ros_ios_chamo/data_receiver/img
exe_addr=../../build/filter_blur/filter_blur
re_name=re_img
if [ -d ${re_name} ]; then
    rm -r ${re_name}
fi
mkdir ${re_name}
for f in $(find ${img_root} -name '*.jpg')
do
    #echo ${f}
    img_name=$(basename ${f})
    output=$(${exe_addr} ${f})
    if (( ${output} > 35 ))
    then
        echo ${img_name}
        cp ${f} ./re_img
    fi
done
