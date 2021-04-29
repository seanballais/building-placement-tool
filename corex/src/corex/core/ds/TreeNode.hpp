#ifndef COREX_CORE_DS_TREE_NODE_HPP
#define COREX_CORE_DS_TREE_NODE_HPP

#include <cstdlib>

#include <EASTL/algorithm.h>
#include <EASTL/array.h>
#include <EASTL/unique_ptr.h>

namespace cx
{
  template <class T, int32_t numChildren>
  struct TreeNode
  {
  public:
    T data;
    TreeNode<T, numChildren>* const parentPtr;

    explicit TreeNode(const T& data,
                      TreeNode<T, numChildren>* const parentPtr = nullptr)
      : data(data)
      , parentPtr(parentPtr)
    {
      eastl::fill(children.begin(), children.end(), nullptr);
    }

    TreeNode() : TreeNode(T{}) {}

    TreeNode<T, numChildren>* getChild(int32_t childNumber)
    {
      return this->children[childNumber].get();
    }

    void setChildValue(int32_t childNumber, T& childData)
    {
      this->children[childNumber] = eastl::make_unique<
        TreeNode<T, numChildren>>(childData, this);
    }

    bool isLeaf()
    {
      return eastl::all_of(
        this->children.begin(),
        this->children.end(),
        [](const eastl::unique_ptr<TreeNode<T, numChildren>>& childPtr) {
          return childPtr == nullptr;
        }
      );
    }
  private:
    eastl::array<eastl::unique_ptr<TreeNode<T, numChildren>>,
                 numChildren> children;
  };
}

#endif
