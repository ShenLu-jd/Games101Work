# Games101作业VS环境配置

## 安装Eigen

官网：http://eigen.tuxfamily.org/index.php?title=Main_Page 

下载最新的zip后解压

解压出来的文件夹名称应该是eigen-version，为了和作业include对应，重命名为eigen3。



## 安装OpenCV

官网：https://opencv.org/releases/

下载最新版安装即可。

安装完成后添加环境变量 `...\include\opencv\build\x64\vc16\bin`

## VS项目环境设置

安装eigen3和opencv库之后，做如下设置。

新建一个C++空项目。

打开项目属性，进行如下设置。

`VC++目录` --> `包含目录`

```
D:\APP\CPPOpenSouceLib    新建eigen3文件夹的上级目录（注意不是eigen3文件夹）
D:\APP\opencv\build\include
```

`VC++目录` --> `库目录`

```
D:\APP\opencv\build\x64\vc16\lib
```

`链接器` --> `输入` --> `附加依赖项` 

```
opencv_world480d.lib
```

`调试环境` --> `环境`

```
PATH=D:\APP\opencv\build\x64\vc16\bin
```



## 重复配置

新建一个项目之后又要重复配置，非常麻烦，可以参考下面的链接，但是`调试环境` --> `环境`的路径，还是需要自己配。

https://blog.csdn.net/weixin_45410343/article/details/108987053