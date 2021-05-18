/**
* \author Xuan Zhang
* \data Mai-July 2018
*/
#ifndef SEMANTICS_JAGUAR_H
#define SEMANTICS_JAGUAR_H

#include <octomap/ColorOcTree.h>
#include <string.h>

namespace octomap
{

  /// Jaguar robot Template Semantic color and label
  struct SemanticsJaguar
  {
    ColorOcTreeNode::Color semantic_color; ///<Semantic color
    float confidence;
    std::string label;

    SemanticsJaguar():semantic_color(), confidence(0.){}

    bool operator==(const SemanticsJaguar& rhs) const
    {
        return semantic_color == rhs.semantic_color
                && confidence == rhs.confidence;
    }

    bool operator!=(const SemanticsJaguar& rhs) const
    {
        return !(*this == rhs);
    }

    ColorOcTreeNode::Color getSemanticColor() const
    {
      return semantic_color;
    }

    std::string getLabel() const 
    {
      return label;
    }

    bool isSemanticsSet() const
    {
      if(semantic_color != ColorOcTreeNode::Color(255,255,255))
        return true;
      return false;
    }

    /// Perform max fusion
    static SemanticsJaguar semanticFusion(const SemanticsJaguar s1, const SemanticsJaguar s2)
    {
      SemanticsJaguar ret;
      // If the same color, update the confidence to the average
      if(s1.semantic_color == s2.semantic_color)
      {
        ret.semantic_color = s1.semantic_color;
        ret.confidence = (s1.confidence + s2.confidence) / 2.;
      }
      // If color is different, keep the larger one and drop a little for the disagreement
      else
      {
        ret = s1.confidence > s2.confidence ? s1 : s2;
        ret.confidence *= 0.9;
      }
      return ret;
    }
  };

  std::ostream& operator<<(std::ostream& out, SemanticsJaguar const& s);
}
#endif //SEMANTICS_MAX_H
