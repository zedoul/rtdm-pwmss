git clone https://github.com/cdsteinkuehler/linux-dev linux-dev
git clone https://github.com/Teknoman117/beaglebot.git

cd linux-dev
git checkout -b bone26-xenomai origin/am33x-v3.8-bone26-xenomai
git checkout 41feff4372d18015bb529554c3451554b984319f
./build_kernel.sh
cd ./KERNEL
git apply ../../beaglebot/encoders/patches/0001-tieqep-driver.patch
cd ../..
./update_kernel.sh
