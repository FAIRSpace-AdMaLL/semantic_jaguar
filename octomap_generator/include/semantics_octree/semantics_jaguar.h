/**
* \author Xuan Zhang
* \data Mai-July 2018
*/
#ifndef SEMANTICS_JAGUAR_H
#define SEMANTICS_JAGUAR_H

#include <octomap/ColorOcTree.h>
#define NUM_SEMANTICS 10
#define EPSILON 0.01


namespace octomap
{

  /// Structure of semantic color with confidence
  struct ColorWithConfidence
  {
    ColorWithConfidence()
    {
      color = ColorOcTreeNode::Color(255,255,255);
      confidence = EPSILON;
    }
    ColorWithConfidence(ColorOcTreeNode::Color col, float conf)
    {
      color = col;
      confidence = conf;
    }

    ColorOcTreeNode::Color color;
    float confidence;

    inline bool operator==(const ColorWithConfidence& rhs) const
    {
        return color == rhs.color && confidence == rhs.confidence;
    }
    inline bool operator!=(const ColorWithConfidence& rhs) const
    {
        return color != rhs.color || confidence != rhs.confidence;
    }
    inline bool operator<(const ColorWithConfidence& rhs) const
    {
      return confidence < rhs.confidence;
    }
    inline bool operator>(const ColorWithConfidence& rhs) const
    {
      return confidence > rhs.confidence;
    }
  };

  std::ostream& operator<<(std::ostream& out, ColorWithConfidence const& c);
  /// Structure contains semantic colors and their confidences
  struct SemanticsJaguar
  {
    std::vector<ColorWithConfidence> data; ///<Semantic colors and confidences, ordered by confidences

    SemanticsJaguar()
    {
      for(int i = 0; i < NUM_SEMANTICS; i++)
      {
        data.push_back(ColorWithConfidence());
      }
    }

    SemanticsJaguar(int n)
    {
      data.resize(n);
      for(int i = 0; i < n; i++)
      {
        data[i] = ColorWithConfidence();
      }
    }

    bool operator==(const SemanticsJaguar& rhs) const
    {
        for(int i = 0; i < NUM_SEMANTICS; i++)
        {
          if(data[i] != rhs.data[i])
          {
            return false;
            break;
          }
        }
        return true;
    }

    bool operator!=(const SemanticsJaguar& rhs) const
    {
        return !(*this == rhs);
    }

    ColorOcTreeNode::Color getSemanticColor() const
    {
      return data[0].color;
    }

    bool isSemanticsSet() const
    {
      for(int i = 0; i < NUM_SEMANTICS; i++)
      {
        if(data[i].color != ColorOcTreeNode::Color(255,255,255))
          return true;
      }
      return false;
    }

    /// Perform bayesian fusion
    static SemanticsJaguar semanticFusion(const SemanticsJaguar s1, const SemanticsJaguar s2);
  };

  std::ostream& operator<<(std::ostream& out, SemanticsJaguar const& s);
}
#endif //SEMANTICS_Jaguar_H
