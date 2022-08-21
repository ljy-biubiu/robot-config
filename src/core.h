/*************************************************************************
	> File Name: robot_config.h
	> Author: ma6174
	> Mail: ma6174@163.com 
	> Created Time: 2018年08月21日 星期二 10时58分57秒
 ************************************************************************/
#ifndef __CORE_H__
#define __CORE_H__


#include <sys/resource.h>
#include<boost/filesystem.hpp>

int setUnlimit() {
	//! 程序崩溃时产生核心转储core来debug程序奔溃原因
	struct rlimit rlmt;
	if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
		return -1;
	}
	printf("Before set rlimit CORE dump current is:%d, max is:%d\n",
			(int)rlmt.rlim_cur, (int)rlmt.rlim_max);

	int CORE_SIZE = 1024*1024*500;

	rlmt.rlim_cur = (rlim_t)CORE_SIZE;
	rlmt.rlim_max = (rlim_t)CORE_SIZE;
	if (setrlimit(RLIMIT_CORE, &rlmt) == -1) {
		return -1;
	}
	if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
		return -1;
	}
	printf("After set rlimit CORE dump current is:%d, max is:%d\n",
			(int)rlmt.rlim_cur, (int)rlmt.rlim_max);
	//! core缓存数设为三个 运行roslaunch一般保存在 $HOME/.ros/下
	std::string HOME(getenv("HOME"));
	if (boost::filesystem::exists(HOME + "/.ros/core")) {
		system("mv core2 core3");
		system("mv core1 core2");
		system("mv core core1");
	}
	return 0;
}


#endif