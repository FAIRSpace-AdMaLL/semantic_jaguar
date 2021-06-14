#include <semantics_octree/semantics_jaguar.h>
#include <vector>
#include <algorithm>

namespace octomap
{

 /*  SemanticsJaguar fuseTop3(const SemanticsJaguar s1, const SemanticsJaguar s2)
  {
    const float alpha = 0.8;

    // Init colors vector with s1 & s2
    std::vector<ColorWithConfidence> semantic_colors = s1.data, s2_colors = s2.data;
    semantic_colors.resize(NUM_SEMANTICS);

    std::cout << s1.data.size() << " " << semantic_colors.size() << " "  << s2_colors.size() << std::endl;


    // Indexes of the affected labels
    std::vector<int> label_ind;

    // Get the last empty position (index without a color/unlabelled index)
    int empty_pos = 0;
    for (auto v :semantic_colors)
    {
      if (v.color == ColorOcTreeNode::Color(255, 255, 255))
      {
        break;
      }
      empty_pos++;
    }
  

    // Normalise
    float s2_sum = SemanticsJaguar::sum(s2_colors);
    for (int i = 0; i < s2_colors.size(); i++)
    {
      s2_colors[i].confidence /= s2_sum;
    }
    float eps = (1 - SemanticsJaguar::sum(s2_colors)) / (NUM_SEMANTICS - 3);

    for(int i = 0; i < s2_colors.size(); i++){

      bool found = false;
      for(int j = 0; j < semantic_colors.size(); j++){


        if(semantic_colors[j].color == s2_colors[i].color){
          semantic_colors[j].confidence *= s2_colors[i].confidence*eps;

          label_ind.push_back(j);
          found = true;
          break;
        }
      }

      if(!found){
        
        // Shouldn't happen!!!
        if(empty_pos >= NUM_SEMANTICS){
          semantic_colors.push_back(ColorWithConfidence(s2_colors[i].color, s2_colors[i].confidence*alpha));
        } else{
          semantic_colors[empty_pos] = ColorWithConfidence(s2_colors[i].color, s2_colors[i].confidence*alpha);   
        }
        empty_pos++;     
      }
    }

    // Update other labels
    for(int i = 0; i <NUM_SEMANTICS; i++){
      if(std::find(label_ind.begin(), label_ind.begin(), i) == label_ind.end()){
        semantic_colors[i].confidence *= eps*alpha;
      }
    }

    float norm = SemanticsJaguar::sum(semantic_colors);

    // Normalize to a probalility distribution
    for (int i = 0; i < semantic_colors.size(); i++)
    {
      semantic_colors[i].confidence /= norm;
      // If confidence is too small, set to epsilon to prevent confidence drop to zero
      if (semantic_colors[i].confidence < EPSILON)
        semantic_colors[i].confidence = EPSILON;
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

  SemanticsJaguar fuseAll(const SemanticsJaguar s1, const SemanticsJaguar s2)
  {

    // Init colors vector with s1
    std::vector<ColorWithConfidence> semantic_colors(NUM_SEMANTICS);
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
      semantic_colors[i] = s1.data[i];
    }
    std::vector<float> confidences2(NUM_SEMANTICS);
    float conf_others1 = 1., conf_others2 = 1.; // Probability for other unknown colors
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
      conf_others1 -= s1.data[i].confidence;
      conf_others2 -= s2.data[i].confidence;
    }

    // Keep a minimum value for others where they are small to be updatable
    if (conf_others1 <= EPSILON)
      conf_others1 = EPSILON;
    if (conf_others2 <= EPSILON)
      conf_others2 = EPSILON;

    // Complete confidences2 vector with s1
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
      bool found = false;
      for (int j = 0; j < NUM_SEMANTICS; j++)
      {
        // If current color is in s2, update confidences2 with confidence of s2
        if (semantic_colors[i].color == s2.data[j].color)
        {
          confidences2[i] = s2.data[j].confidence;
          found = true;
          break;
        }
      }
      // If current color is not in s2, update confidence2 with alpha*conf_others2, also update conf_others2
      if (!found)
      {
        confidences2[i] = 0.8 * conf_others2;
        conf_others2 -= 0.8 * conf_others2;
      }
    }
    // Complete semantic_colors, confidences2 with s2
    for (int i = 0; i < NUM_SEMANTICS; i++)
    {
      bool found = false;
      for (int j = 0; j < NUM_SEMANTICS; j++)
      {
        if (s2.data[i].color == semantic_colors[j].color)
        {
          found = true;
          break;
        }
      }
      // Found new color in s2
      if (!found)
      {
        semantic_colors.push_back(ColorWithConfidence(s2.data[i].color, 0.8 * conf_others1)); // Add to semantic colors
        conf_others1 -= 0.8 * conf_others1;                                                 // Update conf_others1
        confidences2.push_back(s2.data[i].confidence);                                        // Update confidences2
      }
    }

    // Now semantic_colors, confidences2 have the same size.
    // Perform bayesian fusion, save new confidences in semantic_colors
    float sum = conf_others1 * conf_others2;
    for (int i = 0; i < semantic_colors.size(); i++)
    {
      semantic_colors[i].confidence *= confidences2[i]; // Un-normalized confidences
      sum += semantic_colors[i].confidence;
    }
    // Normalize to a probalility distribution
    for (int i = 0; i < semantic_colors.size(); i++)
    {
      semantic_colors[i].confidence /= sum;
      // If confidence is too small, set to epsilon to prevent confidence drop to zero
      if (semantic_colors[i].confidence < EPSILON)
        semantic_colors[i].confidence = EPSILON;
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
 */
  
  
  // Struct ColorWithConfidence implementation -------------------------------------
  std::ostream &operator<<(std::ostream &out, ColorWithConfidence const &c)
  {
    return out << '(' << c.color << ' ' << c.confidence << ')';
  }

  // Struct SemanticsJaguar implementation  --------------------------------------
  SemanticsJaguar SemanticsJaguar::semanticFusion(const SemanticsJaguar s1, const SemanticsJaguar s2)
  {
    // Init colors vector with s1
    std::vector<ColorWithConfidence> semantic_colors(NUM_SEMANTICS);
    for(int i = 0; i < NUM_SEMANTICS; i++)
    {
      semantic_colors[i] = s1.data[i];
    }
    std::vector<float> confidences2(NUM_SEMANTICS);
    float conf_others1 = 1., conf_others2 = 1.; // Probability for other unknown colors
    for(int i = 0; i < NUM_SEMANTICS; i++)
    {
      conf_others1 -= s1.data[i].confidence;
      conf_others2 -= s2.data[i].confidence;
    }

    // Keep a minimum value for others where they are small to be updatable
    if(conf_others1 <= EPSILON)
      conf_others1 = EPSILON;
    if(conf_others2 <= EPSILON)
      conf_others2 = EPSILON;

    // Complete confidences2 vector with s1
    for(int i = 0; i < NUM_SEMANTICS; i++)
    {
      bool found = false;
      for(int j = 0; j < NUM_SEMANTICS; j++)
      {
        // If current color is in s2, update confidences2 with confidence of s2
        if(semantic_colors[i].color == s2.data[j].color)
        {
          confidences2[i] = s2.data[j].confidence;
          found = true;
          break;
        }
      }
      // If current color is not in s2, update confidence2 with alpha*conf_others2, also update conf_others2
      if(!found)
      {
        confidences2[i] = 0.8 * conf_others2;
        conf_others2 -= 0.8 * conf_others2;
      }
    }
    // Complete semantic_colors, confidences2 with s2
    for(int i = 0; i < NUM_SEMANTICS; i++)
    {
      bool found = false;
      for(int j = 0; j < NUM_SEMANTICS; j++)
      {
        if(s2.data[i].color == semantic_colors[j].color)
        {
          found = true;
          break;
        }
      }
      // Found new color in s2
      if(!found)
      {
        semantic_colors.push_back(ColorWithConfidence(s2.data[i].color, 0.8 * conf_others1)); // Add to semantic colors
        conf_others1 -= 0.8 * conf_others1; // Update conf_others1
        confidences2.push_back(s2.data[i].confidence); // Update confidences2
      }
    }
    // Now semantic_colors, confidences2 have the same size.
    // Perform bayesian fusion, save new confidences in semantic_colors
    float sum = conf_others1 * conf_others2;
    for(int i = 0; i < semantic_colors.size(); i++)
    {
      semantic_colors[i].confidence *= confidences2[i]; // Un-normalized confidences
      sum += semantic_colors[i].confidence;
    }
    // Normalize to a probalility distribution
    for(int i = 0; i < semantic_colors.size(); i++)
    {
      semantic_colors[i].confidence /= sum;
      // If confidence is too small, set to epsilon to prevent confidence drop to zero
      if(semantic_colors[i].confidence < EPSILON)
        semantic_colors[i].confidence = EPSILON;
    }

    
    // Keep top NUM_SEMANTICS colors and confidences
    std::sort(semantic_colors.begin(), semantic_colors.end()); // Asc order sorting
    SemanticsJaguar ret;
    for(int i = 0; i < NUM_SEMANTICS; i++)
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
