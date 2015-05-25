for 32 bit only

参考
http://icl.cs.utk.edu/lapack-for-windows/lapack/#libraries_mingw
http://icl.cs.utk.edu/lapack-for-windows/libraries/VisualStudio/3.5.0/Dynamic-MINGW/

1、cblas.lib是自己制作的，解压cblas.tgz文件。将在cblas_f77.h中添加#define ADD_，并创建项目编译所有.c文件
2、将所有dll添加到Windows/SysWOW64文件夹中
3、将include中lapacke.h文件开始出添加：
	#define HAVE_LAPACK_CONFIG_H

	#define LAPACK_COMPLEX_STRUCTURE
4、添加include和lib中的文件分别拷贝到C:\include\Lapack和C:\lib\Lapack