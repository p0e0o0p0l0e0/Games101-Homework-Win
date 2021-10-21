作业7：
1 完成了作业的前三项，实现多线程Path Tracing，提交结果在images中，未实现Microfacet材质渲染。
2 在CMakeLists.txt中引用openmp库，并在get_random_float()方法里添加thread_local指示符实现多线程执行。
3 结果：分辨率为784*784，采样数32，时间为64s。