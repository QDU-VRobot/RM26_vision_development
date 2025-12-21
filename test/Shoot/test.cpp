#include <iostream>
#include "../../rm-main/include/ShootTable.hpp"

ShootTable::TableConfig tableconfig(10,0,2,-1,0.01,"/home/king/AUTO-Aming-system/config/infantry_10_table.bin");
ShootTable table(tableconfig);

int main()
{
    table.Init();
    auto ans = table.Check(4, 0.1);
    std::cout << ans.pitch * (180/M_PI) << " " << ans.t <<"\n";

}