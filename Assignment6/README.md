��ҵ6��BVH��غ�SAH���Ҿ�����ɡ�
�ύ���ͼ/images/binary.ppm��
��ǰ����Ϊ32��Buckets��SAH���ҡ�

�����޸ģ�
BVH.cpp:
1 BVHAccel(), ���splitMethod�жϣ����ΪNAIVE�����ԭ�к���recursiveBuild()�����ΪSAH����recursiveSAHBuild()
2 �޸�getIntersection����ΪgetBVHIntersection����Object���ཻ���������֣�����ý�����ж��߼�������bvh�����жϽ��㣬���ͨ�������ж��Ƿ������ҽڵ㣬�ݹ���㽻�㡣
3 ���CalculateCost(), recursiveSAHBuild(), ����SAH���ҡ�


���ۣ�
������ƽ�ֲ��ң�
����ʱ��Ϊ0s����Ⱦʱ��Ϊ2s��

SAH���ң�buckets����
Ϊ8ʱ��
����ʱ��1s����Ⱦʱ��Ϊ3s��
Ϊ32ʱ��
����ʱ��Ϊ1s����Ⱦʱ��Ϊ4s��

��û�еõ�ʹ��SAH����Ч�ʸ��ߵĽ��ۡ�

����ڼ�����Сcostʱֻ�ڵ�һ��ѡ��һ����ᣬ������x��y��z�ᶼ�����õ���Сcost��buckets����
Ϊ8ʱ��
����ʱ��Ϊ0s����Ⱦʱ��Ϊ2s��
Ϊ32ʱ��
����ʱ��Ϊ0s����Ⱦʱ��Ϊ2s��

���������ֻ�ڵ�һ��ѡ��һ����������Сcost����Ч�ʸ��ߣ���������BVH.cpp L:148��