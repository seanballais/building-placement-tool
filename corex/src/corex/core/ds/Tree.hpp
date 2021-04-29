#ifndef COREX_CORE_DS_TREE_HPP
#define COREX_CORE_DS_TREE_HPP

#include <EASTL/unique_ptr.h>

#include <corex/core/ds/TreeNode.hpp>

namespace cx
{
  template <class T, int32_t numChildrenPerNode>
  class Tree
  {
  public:
    Tree() : root(eastl::make_unique<TreeNode<T, numChildrenPerNode>>()) {}

    TreeNode<T, numChildrenPerNode>* getRoot()
    {
      return root.get();
    }

  private:
    eastl::unique_ptr<TreeNode<T, numChildrenPerNode>> root;
  };
}

#endif
