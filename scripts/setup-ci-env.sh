#!/usr/bin/env bash

cd $APP_ROOT

mkdir $APP_ROOT/scripts


mv $GITHUB_WORKSPACE/* $APP_ROOT
mv $GITHUB_WORKSPACE/.[!.]* $APP_ROOT

chmod 777 -R $APP_ROOT

git config --global --add safe.directory "*"
git config --global core.sshCommand "ssh -o StrictHostKeyChecking=accept-new"

echo "ZEPHYR_SDK_INSTALL_DIR=/opt/toolchains/zephyr-sdk-$ZSDK_VERSION" >> $GITHUB_ENV
west update

