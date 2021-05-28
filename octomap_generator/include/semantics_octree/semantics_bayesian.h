/**
* \author Xuan Zhang
* \data Mai-July 2018
*/
#ifndef SEMANTICS_BAYESIAN_H
#define SEMANTICS_BAYESIAN_H

#include <octomap/ColorOcTree.h>
#include <semantics_octree/semantics_jaguar.h>
#define NUM_SEMANTICS_BAYESIAN 10


namespace octomap
{

  /// Structure contains semantic colors and their confidences
  struct SemanticsBayesian
  {
    ColorWithConfidence data[NUM_SEMANTICS_BAYESIAN]; ///<Semantic colors and confidences, ordered by confidences

    SemanticsBayesian()
    {
      for(int i = 0; i < NUM_SEMANTICS_BAYESIAN; i++)
      {
        data[i] = ColorWithConfidence();
      }
    }

    bool operator==(const SemanticsBayesian& rhs) const
    {
        for(int i = 0; i < NUM_SEMANTICS_BAYESIAN; i++)
        {
          if(data[i] != rhs.data[i])
          {
            return false;
            break;
          }
        }
        return true;
    }

    bool operator!=(const SemanticsBayesian& rhs) const
    {
        return !(*this == rhs);
    }

    ColorOcTreeNode::Color getSemanticColor() const
    {
      return data[0].color;
    }

    bool isSemanticsSet() const
    {
      for(int i = 0; i < NUM_SEMANTICS_BAYESIAN; i++)
      {
        if(data[i].color != ColorOcTreeNode::Color(255,255,255))
          return true;
      }
      return false;
    }

    /// Perform bayesian fusion
    static SemanticsBayesian semanticFusion(const SemanticsBayesian s1, const SemanticsBayesian s2);
  };

  std::ostream& operator<<(std::ostream& out, SemanticsBayesian const& s);
}
#endif //SEMANTICS_BAYESIAN_H
