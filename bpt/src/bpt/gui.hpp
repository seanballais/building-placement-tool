#ifndef BPT_GUI_HPP
#define BPT_GUI_HPP

#include <type_traits>

#include <imgui.h>

#include <bpt/utils.hpp>

namespace bpt
{
  template <
    typename T,
    typename U,
    size_t N,
    std::enable_if_t<std::is_same_v<T, U>, bool> = true
  > void drawComboBox(const char* label, T& targetVariable, const U (&items)[N])
  {
    if (ImGui::BeginCombo(label, castToCString(targetVariable))) {
      for (T item : items) {
        bool isItemSelected = (targetVariable == item);
        if (ImGui::Selectable(castToCString(item), isItemSelected)) {
          targetVariable = item;
        }

        if (isItemSelected) {
          ImGui::SetItemDefaultFocus();
        }
      }

      ImGui::EndCombo();
    }
  }
}

#endif
