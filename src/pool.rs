//! [Pool] implementation.

use std::{
    array::from_fn,
    ops::{Index, IndexMut},
};

use crate::{
    bounding::{Aabb, Unsigned},
    node::{Node, NodeType},
    ElementId, NodeId, Position, TreeError,
};

/// Pool data structure.
/// When element is removed no memory deallocation happens.
/// Removed elements are only marked as deleted and their memory could be reused.  
pub struct Pool<T> {
    pub(crate) vec: Vec<T>,
    pub(crate) garbage: Vec<usize>,
}

impl<U: Unsigned> Default for Pool<Node<U>> {
    fn default() -> Self {
        let root = Node::default();
        let vec = vec![root];

        Pool {
            vec,
            garbage: Default::default(),
        }
    }
}

impl<T: Position> Default for Pool<T> {
    fn default() -> Self {
        Pool {
            vec: Default::default(),
            garbage: Default::default(),
        }
    }
}

impl Default for Pool<NodeId> {
    fn default() -> Self {
        Pool {
            vec: Default::default(),
            garbage: Default::default(),
        }
    }
}

/// Indexing a [`pool`](Pool) of [`nodes`](Node) with [`NodeId`]
///
/// ```no_run
/// let node = &tree.nodes[NodeId(42)];
/// // let node = &tree.nodes[ElementId(42)]; // Error
/// ```
impl<U: Unsigned> Index<NodeId> for Pool<Node<U>> {
    type Output = Node<U>;

    fn index(&self, index: NodeId) -> &Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Indexing garbaged node: {index}"
        );
        self.get_unchecked(index)
    }
}

/// Mutable Indexing a [`pool`](Pool) of [`nodes`](Node) with [`NodeId`]
///
/// ```no_run
/// let mut node = &mut tree.nodes[NodeId(42)];
/// // let mut node = &mut tree.nodes[ElementId(42)]; // Error
/// ```
impl<U: Unsigned> IndexMut<NodeId> for Pool<Node<U>> {
    fn index_mut(&mut self, index: NodeId) -> &mut Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Mut Indexing garbaged node: {index}"
        );
        self.get_mut_unchecked(index)
    }
}

/// Indexing a [`pool`](Pool) of `T: Position` with [`ElementId`]
///
/// ```no_run
/// let element = &tree.element[ElementId(42)];
/// // let element = &tree.element[NodeId(42)]; // Error
/// ```
impl<T: Position> Index<ElementId> for Pool<T> {
    type Output = T;

    fn index(&self, index: ElementId) -> &Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Indexing garbaged element: {index}"
        );
        self.get_unchecked(index)
    }
}

/// Mutable Indexing a [`pool`](Pool) of `T: Position` with [`ElementId`]
///
/// ```no_run
/// let mut element = &mut tree.element[ElementId(42)];
/// // let mut element = &mut tree.element[NodeId(42)]; // Error
/// ```
impl<T: Position> IndexMut<ElementId> for Pool<T> {
    fn index_mut(&mut self, index: ElementId) -> &mut Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Mut Indexing garbaged element: {index}"
        );
        self.get_mut_unchecked(index)
    }
}

/// Indexing a [`pool`](Pool) of [`node ids`](NodeId) with [`ElementId`]
///
/// ```no_run
/// let node_id = &tree.map[ElementId(42)];
/// // let node_id = &tree.map[NodeId(42)]; // Error
/// ```
impl Index<ElementId> for Pool<NodeId> {
    type Output = NodeId;

    fn index(&self, index: ElementId) -> &Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Indexing garbaged element: {index}"
        );
        self.get_unchecked(index)
    }
}

/// Mutable Indexing a [`pool`](Pool) of [`node ids`](NodeId) with [`ElementId`]
///
/// ```no_run
/// let mut node_id = &mut tree.map[ElementId(42)];
/// // let mut node_id = &mut tree.map[NodeId(42)]; // Error
/// ```
impl IndexMut<ElementId> for Pool<NodeId> {
    fn index_mut(&mut self, index: ElementId) -> &mut Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Mut Indexing garbaged element: {index}"
        );
        self.get_mut_unchecked(index)
    }
}

impl<T> Pool<T> {
    fn _insert(&mut self, t: T) -> usize {
        if let Some(idx) = self.garbage.pop() {
            self.vec[idx] = t;
            idx
        } else {
            self.vec.push(t);
            self.vec.len() - 1
        }
    }

    /// Returns the mumber of actual elements.
    ///
    /// Elements marked as deleted are not counted.
    pub fn len(&self) -> usize {
        self.vec.len() - self.garbage_len()
    }

    /// Is the pool is empty.
    ///
    /// Elements marked as deleted are not counted.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the number of deleted elements.
    pub fn garbage_len(&self) -> usize {
        self.garbage.len()
    }

    /// Returns a [PoolIterator], which iterates over an actual elements.
    ///
    /// Elements marked as deleted are skipped.
    pub fn iter(&self) -> PoolIterator<T> {
        PoolIterator::new(self)
    }
}

impl<U: Unsigned> Pool<Node<U>> {
    /// Construct a [`Pool`] of [`nodes`](Node) from [`Aabb`].
    ///
    /// Node will adopt aabb's dimensions.
    pub(crate) fn from_aabb(aabb: Aabb<U>) -> Self {
        let root = Node::from_aabb(aabb, None);
        let vec = vec![root];
        Pool {
            vec,
            garbage: Default::default(),
        }
    }

    /// Construct a [`Pool`] of [`nodes`](Node).
    ///
    /// Helps to reduce the amount of the memory reallocations.
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        let root = Node::default();
        let mut vec = Vec::with_capacity(capacity);
        vec.push(root);

        Pool {
            vec,
            garbage: Default::default(),
        }
    }

    /// Construct a [`Pool`] of [`nodes`](Node) from [`Aabb`] with capacity.
    ///
    /// Node will adopt aabb's dimensions.
    /// Helps to reduce the amount of the memory reallocations.
    pub(crate) fn from_aabb_with_capacity(aabb: Aabb<U>, capacity: usize) -> Self {
        let root = Node::from_aabb(aabb, None);
        let mut vec = Vec::with_capacity(capacity);
        vec.push(root);

        Pool {
            vec,
            garbage: Default::default(),
        }
    }

    pub(crate) fn insert(&mut self, t: Node<U>) -> NodeId {
        self._insert(t).into()
    }

    pub(crate) fn remove(&mut self, node: NodeId) {
        self.garbage.push(node.into());
    }

    pub(crate) fn branch(&mut self, parent: NodeId) -> [NodeId; 8] {
        let aabbs = self[parent].aabb.split();
        from_fn(|i| self.insert(Node::from_aabb(aabbs[i], Some(parent))))
    }

    pub(crate) fn collapse(
        &mut self,
        parent: Option<NodeId>,
    ) -> Result<Option<(ElementId, NodeId)>, TreeError> {
        if let Some(parent) = parent {
            let mut p = self[parent];

            match p.ntype {
                NodeType::Branch(ref mut branch) => {
                    branch.decrement();
                    match branch.filled {
                        0 => {
                            let children = branch.children;
                            p.ntype = NodeType::Empty;
                            self[parent] = p;
                            children.map(|child| self.remove(child));
                            return self.collapse(p.parent);
                        }

                        1 => {
                            for child in branch.children {
                                let c = self[child];
                                match c.ntype {
                                    NodeType::Leaf(element) => {
                                        let children = branch.children;
                                        p.ntype = NodeType::Leaf(element);
                                        self[parent] = p;
                                        children.map(|child| self.remove(child));
                                        return Ok(Some((element, parent)));
                                    }
                                    NodeType::Branch(_) => break,
                                    NodeType::Empty => (),
                                }
                            }
                        }

                        _ => (),
                    }
                }
                _ => {
                    return Err(TreeError::NotBranch(format!(
                        "Attempt to collapse a node of type {}",
                        p.ntype
                    )))
                }
            }

            self[parent] = p;
        }
        Ok(None)
    }

    pub fn get(&self, node: NodeId) -> Option<&Node<U>> {
        if !self.garbage.contains(&(node.0 as usize)) {
            self.vec.get(node.0 as usize)
        } else {
            None
        }
    }

    pub fn get_mut(&mut self, node: NodeId) -> Option<&mut Node<U>> {
        if !self.garbage.contains(&(node.0 as usize)) {
            self.vec.get_mut(node.0 as usize)
        } else {
            None
        }
    }

    pub fn get_unchecked(&self, node: NodeId) -> &Node<U> {
        &self.vec[node.0 as usize]
    }

    pub fn get_mut_unchecked(&mut self, node: NodeId) -> &mut Node<U> {
        &mut self.vec[node.0 as usize]
    }
}

impl<T: Position> Pool<T> {
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Pool {
            vec: Vec::with_capacity(capacity),
            garbage: Default::default(),
        }
    }

    pub(crate) fn insert(&mut self, t: T) -> ElementId {
        self._insert(t).into()
    }

    pub(crate) fn remove(&mut self, element: ElementId) {
        self.garbage.push(element.into());
    }

    pub fn get(&self, element: ElementId) -> Option<&T> {
        if !self.garbage.contains(&(element.0 as usize)) {
            self.vec.get(element.0 as usize)
        } else {
            None
        }
    }

    pub fn get_mut(&mut self, element: ElementId) -> Option<&mut T> {
        if !self.garbage.contains(&(element.0 as usize)) {
            self.vec.get_mut(element.0 as usize)
        } else {
            None
        }
    }

    pub fn get_unchecked(&self, element: ElementId) -> &T {
        &self.vec[element.0 as usize]
    }

    pub fn get_mut_unchecked(&mut self, element: ElementId) -> &mut T {
        &mut self.vec[element.0 as usize]
    }
}

impl Pool<NodeId> {
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Pool {
            vec: Vec::with_capacity(capacity),
            garbage: Default::default(),
        }
    }

    pub(crate) fn insert(&mut self, t: NodeId) -> ElementId {
        self._insert(t).into()
    }

    pub(crate) fn remove(&mut self, element: ElementId) {
        self.garbage.push(element.into());
    }

    pub fn get(&self, element: ElementId) -> Option<&NodeId> {
        if !self.garbage.contains(&(element.0 as usize)) {
            self.vec.get(element.0 as usize)
        } else {
            None
        }
    }

    pub fn get_mut(&mut self, element: ElementId) -> Option<&mut NodeId> {
        if !self.garbage.contains(&(element.0 as usize)) {
            self.vec.get_mut(element.0 as usize)
        } else {
            None
        }
    }

    pub fn get_unchecked(&self, element: ElementId) -> &NodeId {
        &self.vec[element.0 as usize]
    }

    pub fn get_mut_unchecked(&mut self, element: ElementId) -> &mut NodeId {
        &mut self.vec[element.0 as usize]
    }
}

/// Iterator for a [Pool].
///
/// Yields only an actual elements.
/// Elements marked as removed are skipped.
pub struct PoolIterator<'pool, T> {
    pool: &'pool Pool<T>,
    current: usize,
}

impl<'pool, T> PoolIterator<'pool, T> {
    fn new(pool: &'pool Pool<T>) -> Self {
        PoolIterator {
            pool,
            current: Default::default(),
        }
    }
}

impl<'pool, T> Iterator for PoolIterator<'pool, T> {
    type Item = &'pool T;

    fn next(&mut self) -> Option<Self::Item> {
        while self.current < self.pool.vec.len() {
            if self.pool.garbage.contains(&self.current) {
                self.current += 1;
                continue;
            } else {
                let current = self.current;
                self.current += 1;
                return Some(&self.pool.vec[current]);
            }
        }
        None
    }
}
