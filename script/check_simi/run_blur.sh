training_img_root=/media/psf/Home/Documents/code/chamo_db/script/filter_cali/re_img
exe_addr=../../build/check_simi/check_simi
if [ -f ./img_list.txt ]; then
    rm img_list.txt
fi

if [ -f group_re.txt ]; then
    rm group_re.txt
fi

for f in $(find ${training_img_root} -name '*.jpg')
do
    echo $(basename ${f}) >> img_list.txt
done

${exe_addr} blur ${training_img_root} img_list.txt >> group_re.txt
