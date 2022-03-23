source ~/STM32MPU_workspace/STM32MP15-Ecosystem-v2.1.0/Developer-Package/SDK/environment-setup-cortexa7t2hf-neon-vfpv4-ostl-linux-gnueabi
echo $ARCH
echo $CROSS_COMPILE
$CC --version
echo $OECORE_SDK_VERSION

make ARCH=arm
