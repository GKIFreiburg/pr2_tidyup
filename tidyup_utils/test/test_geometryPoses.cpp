#include "tidyup_utils/geometryPoses.h"
#include <stdlib.h>

int main(int argc, char** argv) 
{
   GeometryPoses poses;
   if(!poses.load(argv[1])) {
      exit(1);
   }

   std::cout << poses << std::endl;

   return 0;
}

