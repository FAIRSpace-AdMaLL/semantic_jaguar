#include<semantics_octree/semantics_jaguar.h>
namespace octomap
{
  std::ostream& operator<<(std::ostream& out, SemanticsJaguar const& s)
  {
    return out << '(' << s.semantic_color << ", " << s.confidence << ", " << s.label <<')';
  }
}
