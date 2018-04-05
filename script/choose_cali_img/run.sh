blur_file=/media/psf/Home/Documents/code/chamo_db/script/filter_cali/blur_est.txt
group_file=/media/psf/Home/Documents/code/chamo_db/script/check_simi/group_re.txt
img_root=/media/psf/Home/Documents/code/chamo_db/script/filter_cali/re_img
exe_addr=../../build/choose_cali_img/choose_cali_img
re_name=re_img
if [ -d ${re_name} ]; then
    rm -r ${re_name}
fi
mkdir ${re_name}
for item in $(${exe_addr} ${blur_file} ${group_file})
do
    cp ${img_root}/${item} ${re_name}
done
