# 固态硬盘闪存转换层
开发一个固态硬盘闪存转换层，管理逻辑地址到物理地址的映射以及对SDD的读、写、抹除操作。 

主要工作： 
- 采用Page-level Mapping 的方式实现逻辑地址到物理地址的映射 
- 采用Cost-Benefit Ratio 的垃圾回收策略，实现对失效数据占据的存储空间的回收，并尽量减少写放大 
- 使用flash block 的erase count 对其cost-benefit ratio 进行调节，以及引入cold data migration，实现wear leveling 
