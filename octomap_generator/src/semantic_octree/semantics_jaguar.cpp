#include <semantics_octree/semantics_jaguar.h>
#include <vector>
#include <algorithm>


namespace octomap
{

  float l2_norm(std::vector<ColorWithConfidence> v)
  {
    float accum = 0.;
    for (int i = 0; i < v.size(); ++i)
    {
      accum += v[i].confidence * v[i].confidence;
    }
    float norm = sqrt(accum);
  }

  // Struct ColorWithConfidence implementation -------------------------------------
  std::ostream &operator<<(std::ostream &out, ColorWithConfidence const &c)
  {
    return out << '(' << c.color << ' ' << c.confidence << ')';
  }

  // Struct SemanticsJaguar implementation  --------------------------------------
  SemanticsJaguar SemanticsJaguar::semanticFusion(const SemanticsJaguar s1, const SemanticsJaguar s2)
  {

    const float alpha = 1;

    // Init colors vector with s1
    std::vector<ColorWithConfidence> semantic_colors(NUM_SEMANTICS), s2_colors = s2.data;
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
      semantic_colors[i] = s1.data[i];
    }

    std::sort(s2_colors.begin(), s2_colors.end()); // Asc order sorting

    float s2_norm = l2_norm(s2_colors);

    for(int i = 0; i < s2_colors.size(); i++){
      s2_colors[i].confidence /= s2_norm;
    }

    float eps = (1 - sum(s2.data))/7; 
    

    // Complete confidences2 vector with s1
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
      bool found = false;
      for (int j = 0; j < 3; j++)
      {
        // If current color is in s2, update confidences2 with confidence of s2
        if (semantic_colors[i].color == s2_colors[s2_colors.size() - 1 - j].color)
        {
          // propogate to instead of equals
          semantic_colors[i].confidence *= s2_colors[s2_colors.size() - 1 - j].confidence * alpha;
          found = true;
          break;
        }
      }
      // If current color is not in s2, update confidence2 with alpha*conf_others2, also update conf_others2
      if (!found)
      {
        semantic_colors[i].confidence *= eps*alpha;
      }
    }

    float norm = l2_norm(semantic_colors);

    // Normalize to a probalility distribution
    for (int i = 0; i < semantic_colors.size(); i++)
    {
      semantic_colors[i].confidence /= norm;
      // If confidence is too small, set to epsilon to prevent confidence drop to zero
      //if (semantic_colors[i].confidence < EPSILON)
      //  semantic_colors[i].confidence = EPSILON;
    }
    // Keep top NUM_SEMANTICS colors and confidences

    std::sort(semantic_colors.begin(), semantic_colors.end()); // Asc order sorting
    SemanticsJaguar ret;

    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
      ret.data[i] = semantic_colors[semantic_colors.size() - 1 - i]; // Take last elements
    }

    return ret;
  }

  std::ostream &operator<<(std::ostream &out, SemanticsJaguar const &s)
  {
    out << '(';
    for (int i = 0; i < NUM_SEMANTICS - 1; i++)
      out << s.data[i] << ' ';
    out << s.data[NUM_SEMANTICS - 1];
    out << ')';
    return out;
  }

} //namespace octomap
