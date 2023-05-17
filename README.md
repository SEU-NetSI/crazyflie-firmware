## 烧录至无人机
* 在代码的`config`目录下，有这样三个文件，分别对应着不同接口模式的dw3000芯片的配置
  ![image](https://user-images.githubusercontent.com/126245721/226831070-8ffa1156-c531-46fc-80fd-0f4cb1c4ca28.png)
  
* 如果dw3000接口是uart2模式，则执行
  ```
  make adhoc_uart2_defconfig
  ```
* 执行`make -j 12`
* 按住无人机开机键（长按3s）进入bootloader模式, 电脑插上PA，执行`make cload` 完成烧录
  

## 运行该项目
* 执行根目录下的`set_uwb_address.py` 脚本，给无人机`ADHOC.MY_UWB_ADDRESS`参数编号，建议编号为0、1、2、3............（注意一定要有0号和1号）
    编号后，可以通过cfclient查看设置的编号：
    ![image](https://user-images.githubusercontent.com/126245721/226830006-ef31ba7c-3a65-4f37-a955-1fd2e6c4d3b0.png)
*******

* 将编号为1的无人机    设置keepflying=1,其他与之进行通信的无人机就全部飞起来了（执行relative_control.c中的飞控逻辑）![img_v2_38e0179a-12ab-480f-a495-8e90674125fg](https://user-images.githubusercontent.com/126245721/226830209-21f08524-4961-4c9a-bba7-41fef374c346.jpg)
