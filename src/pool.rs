//! [`Pool`] implementation.

use std::{
    array::from_fn,
    iter::Enumerate,
    ops::{Index, IndexMut},
};

use smallvec::SmallVec;

use crate::{
    bounding::{Aabb, Unsigned},
    node::{Node, NodeType},
    ElementId, NodeId, TreeError, Volume,
};

/// [`PoolItem`] data structure that combines both the garbage flag
/// and the actual item together for better cache locality.
#[derive(Clone)]
pub(crate) struct PoolItem<T> {
    pub(crate) item: T,
    pub(crate) garbage: bool,
}
impl<T> From<T> for PoolItem<T> {
    fn from(item: T) -> Self {
        PoolItem {
            item,
            garbage: false,
        }
    }
}

impl<T: std::fmt::Debug> std::fmt::Debug for PoolItem<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PoolItem")
            .field("item", &self.item)
            .field("garbage", &self.garbage)
            .finish()
    }
}

/// [`Pool`] data structure.
///
/// When element is removed no memory deallocation happens.
/// Removed elements are only marked as deleted and their memory could be reused.  
#[derive(Clone)]
pub struct Pool<T> {
    pub(crate) vec: Vec<PoolItem<T>>,
    pub(crate) garbage: Vec<usize>,
}

impl<U: Unsigned> Default for Pool<Node<U>> {
    fn default() -> Self {
        let root = Node::default();
        let vec = vec![root.into()];

        Pool {
            vec,
            garbage: Default::default(),
        }
    }
}
impl<U: Unsigned> Pool<Node<U>> {
    /// Clears all the items in the pool
    pub fn clear(&mut self) {
        self.vec.clear();
        self.vec.push(Node::default().into());
        self.garbage.clear();
    }

    /// Clears all the items in the pool and initiates it with an aabb.
    pub fn clear_with_aabb(&mut self, aabb: Aabb<U>) {
        self.vec.clear();
        self.vec.push(Node::from_aabb(aabb, None).into());
        self.garbage.clear();
    }
}

impl<T: Volume> Default for Pool<T> {
    fn default() -> Self {
        Pool {
            vec: Default::default(),
            garbage: Default::default(),
        }
    }
}
impl<T: Volume> Pool<T> {
    /// Clears all the items in the pool
    pub fn clear(&mut self) {
        self.vec.clear();
        self.garbage.clear();
    }
}

impl<T: std::fmt::Debug> std::fmt::Debug for Pool<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Pool")
            .field("vec", &self.vec)
            .field("garbage", &self.garbage)
            .finish()
    }
}

impl Default for Pool<SmallVec<[NodeId; 1]>> {
    fn default() -> Self {
        Pool {
            vec: Default::default(),
            garbage: Default::default(),
        }
    }
}
impl Pool<SmallVec<[NodeId; 1]>> {
    /// Clears all the items in the pool
    pub fn clear(&mut self) {
        self.vec.clear();
        self.garbage.clear();
    }
}

/// Indexing a [`pool`](Pool) of [`nodes`](Node) with [`NodeId`]
///
/// ```ignore
/// let node = &tree.nodes[NodeId(42)];
/// // let node = &tree.nodes[ElementId(42)]; // Error
/// ```
impl<U: Unsigned> Index<NodeId> for Pool<Node<U>> {
    type Output = Node<U>;

    fn index(&self, index: NodeId) -> &Self::Output {
        debug_assert!(!self.is_garbaged(index), "Indexing garbaged node: {index}");
        self.get_unchecked(index)
    }
}

/// Mutable Indexing a [`pool`](Pool) of [`nodes`](Node) with [`NodeId`]
///
/// ```ignore
/// let mut node = &mut tree.nodes[NodeId(42)];
/// // let mut node = &mut tree.nodes[ElementId(42)]; // Error
/// ```
impl<U: Unsigned> IndexMut<NodeId> for Pool<Node<U>> {
    fn index_mut(&mut self, index: NodeId) -> &mut Self::Output {
        debug_assert!(
            !self.is_garbaged(index),
            "Mut Indexing garbaged node: {index}"
        );
        self.get_mut_unchecked(index)
    }
}

/// Indexing a [`pool`](Pool) of `T: Position` with [`ElementId`]
///
/// ```ignore
/// let element = &tree.element[ElementId(42)];
/// // let element = &tree.element[NodeId(42)]; // Error
/// ```
impl<T: Volume> Index<ElementId> for Pool<T> {
    type Output = T;

    fn index(&self, index: ElementId) -> &Self::Output {
        debug_assert!(
            !self.is_garbaged(index),
            "Indexing garbaged element: {index}"
        );
        self.get_unchecked(index)
    }
}

/// Mutable Indexing a [`pool`](Pool) of `T: Position` with [`ElementId`]
///
/// ```ignore
/// let mut element = &mut tree.element[ElementId(42)];
/// // let mut element = &mut tree.element[NodeId(42)]; // Error
/// ```
impl<T: Volume> IndexMut<ElementId> for Pool<T> {
    fn index_mut(&mut self, index: ElementId) -> &mut Self::Output {
        debug_assert!(
            !self.is_garbaged(index),
            "Mut Indexing garbaged element: {index}"
        );
        self.get_mut_unchecked(index)
    }
}

/// Indexing a [`pool`](Pool) of [`node ids`](NodeId) with [`ElementId`]
///
/// ```ignore
/// let node_id = &tree.map[ElementId(42)];
/// // let node_id = &tree.map[NodeId(42)]; // Error
/// ```
impl Index<ElementId> for Pool<NodeId> {
    type Output = NodeId;

    fn index(&self, index: ElementId) -> &Self::Output {
        debug_assert!(
            !self.is_garbaged(index),
            "Indexing garbaged element: {index}"
        );
        self.get_unchecked(index)
    }
}

/// Mutable Indexing a [`pool`](Pool) of [`node ids`](NodeId) with [`ElementId`]
///
/// ```ignore
/// let mut node_id = &mut tree.map[ElementId(42)];
/// // let mut node_id = &mut tree.map[NodeId(42)]; // Error
/// ```
impl IndexMut<ElementId> for Pool<NodeId> {
    fn index_mut(&mut self, index: ElementId) -> &mut Self::Output {
        debug_assert!(
            !self.is_garbaged(index),
            "Mut Indexing garbaged element: {index}"
        );
        self.get_mut_unchecked(index)
    }
}

impl<T> Pool<T> {
    #[inline(always)]
    fn _insert(&mut self, t: T) -> usize {
        if let Some(idx) = self.garbage.pop() {
            self.vec[idx].garbage = false;
            self.vec[idx].item = t;
            idx
        } else {
            self.vec.push(t.into());
            self.vec.len() - 1
        }
    }

    /// Restores all the garbage elements back to real elements. Effectively
    /// this is a rollback of all the remove operations that happened
    pub fn restore_garbage(&mut self) {
        for idx in self.garbage.drain(..) {
            self.vec[idx].garbage = false;
        }
    }

    /// Returns the number of actual elements.
    ///
    /// Elements marked as deleted are not counted.
    #[inline(always)]
    pub fn len(&self) -> usize {
        self.vec.len() - self.garbage_len()
    }

    /// Is the pool is empty.
    ///
    /// Elements marked as deleted are not counted.
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the number of deleted elements.
    #[inline(always)]
    pub fn garbage_len(&self) -> usize {
        self.garbage.len()
    }

    /// Returns a [`PoolIterator`], which iterates over an actual elements.
    ///
    /// Elements marked as deleted are skipped.
    pub fn iter(&self) -> PoolIterator<T> {
        PoolIterator::new(self)
    }

    /// Returns a [`PoolIterator`], which iterates over an actual elements and element ids
    ///
    /// Elements marked as deleted are skipped.
    pub fn iter_elements(&self) -> PoolElementIterator<T> {
        PoolElementIterator::new(self)
    }
}

impl<T> IntoIterator for Pool<T> {
    type Item = T;
    type IntoIter = PoolIntoIterator<T>;

    fn into_iter(self) -> Self::IntoIter {
        PoolIntoIterator::new(self)
    }
}

impl<U: Unsigned> Pool<Node<U>> {
    /// Construct a [`Pool`] of [`nodes`](Node) from [`Aabb`].
    ///
    /// Node will adopt aabb's dimensions.
    pub(crate) fn from_aabb(aabb: Aabb<U>) -> Self {
        let root = Node::from_aabb(aabb, None);
        let vec = vec![root.into()];
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
        vec.push(root.into());

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
        vec.push(root.into());

        Pool {
            vec,
            garbage: Default::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn insert(&mut self, t: Node<U>) -> NodeId {
        self._insert(t).into()
    }

    #[inline(always)]
    pub(crate) fn remove(&mut self, node: NodeId) {
        let index: usize = node.into();
        self.vec[index].garbage = true;
        self.garbage.push(index);
    }

    #[inline(always)]
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

    #[inline(always)]
    pub fn get(&self, node: NodeId) -> Option<&Node<U>> {
        if !self.is_garbaged(node) {
            self.vec.get(node.0 as usize).map(|node| &node.item)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_mut(&mut self, node: NodeId) -> Option<&mut Node<U>> {
        if !self.is_garbaged(node) {
            self.vec.get_mut(node.0 as usize).map(|node| &mut node.item)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_unchecked(&self, node: NodeId) -> &Node<U> {
        &self.vec[node.0 as usize].item
    }

    #[inline(always)]
    pub fn get_mut_unchecked(&mut self, node: NodeId) -> &mut Node<U> {
        &mut self.vec[node.0 as usize].item
    }

    #[inline(always)]
    pub fn is_garbaged(&self, node: NodeId) -> bool {
        self.vec[node.0 as usize].garbage
    }
}

impl<T: Volume> Pool<T> {
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Pool {
            vec: Vec::with_capacity(capacity),
            garbage: Default::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn insert(&mut self, t: T) -> ElementId {
        self._insert(t).into()
    }

    #[inline(always)]
    pub(crate) fn remove(&mut self, element: ElementId) {
        let index: usize = element.into();
        if !self.vec[index].garbage {
            self.vec[index].garbage = true;
            self.garbage.push(index);
        }
    }

    #[inline(always)]
    pub fn get(&self, element: ElementId) -> Option<&T> {
        if !self.is_garbaged(element) {
            self.vec.get(element.0 as usize).map(|item| &item.item)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_mut(&mut self, element: ElementId) -> Option<&mut T> {
        if !self.is_garbaged(element) {
            self.vec
                .get_mut(element.0 as usize)
                .map(|item| &mut item.item)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_unchecked(&self, element: ElementId) -> &T {
        &self.vec[element.0 as usize].item
    }

    #[inline(always)]
    pub fn get_mut_unchecked(&mut self, element: ElementId) -> &mut T {
        &mut self.vec[element.0 as usize].item
    }

    #[inline(always)]
    pub fn is_garbaged(&self, element: ElementId) -> bool {
        let idx: usize = element.into();
        self.vec[idx].garbage
    }

    #[inline(always)]
    pub fn has_garbage(&self) -> bool {
        !self.garbage.is_empty()
    }
}

impl Pool<NodeId> {
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Pool {
            vec: Vec::with_capacity(capacity),
            garbage: Default::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn insert(&mut self, t: NodeId) -> ElementId {
        self._insert(t).into()
    }

    #[inline(always)]
    pub(crate) fn remove(&mut self, element: ElementId) {
        let index: usize = element.into();
        self.vec[index].garbage = true;
        self.garbage.push(index);
    }

    #[inline(always)]
    pub fn get(&self, element: ElementId) -> Option<&NodeId> {
        if !self.is_garbaged(element) {
            self.vec.get(element.0 as usize).map(|item| &item.item)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_mut(&mut self, element: ElementId) -> Option<&mut NodeId> {
        if !self.is_garbaged(element) {
            self.vec
                .get_mut(element.0 as usize)
                .map(|item| &mut item.item)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_unchecked(&self, element: ElementId) -> &NodeId {
        &self.vec[element.0 as usize].item
    }

    #[inline(always)]
    pub fn get_mut_unchecked(&mut self, element: ElementId) -> &mut NodeId {
        &mut self.vec[element.0 as usize].item
    }

    #[inline(always)]
    pub fn is_garbaged(&self, element: ElementId) -> bool {
        self.vec[element.0 as usize].garbage
    }
}

/// Iterator for a [`Pool`].
///
/// Yields only an actual elements.
/// Elements marked as removed are skipped.
#[derive(Clone)]
pub struct PoolIterator<'pool, T> {
    inner: std::slice::Iter<'pool, PoolItem<T>>,
    garbage_len: usize,
}

impl<'pool, T> PoolIterator<'pool, T> {
    fn new(pool: &'pool Pool<T>) -> Self {
        PoolIterator {
            inner: pool.vec.iter(),
            garbage_len: pool.garbage_len(),
        }
    }
}

impl<'pool, T> Iterator for PoolIterator<'pool, T> {
    type Item = &'pool T;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let next = self.inner.next()?;
            if !next.garbage {
                return Some(&next.item);
            }
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let hint = self.inner.size_hint();
        (
            hint.0.saturating_sub(self.garbage_len),
            hint.1.map(|x| x.saturating_sub(self.garbage_len)),
        )
    }
}

impl<T> DoubleEndedIterator for PoolIterator<'_, T> {
    fn next_back(&mut self) -> Option<Self::Item> {
        loop {
            let next = self.inner.next_back()?;
            if !next.garbage {
                return Some(&next.item);
            }
        }
    }
}

impl<T> ExactSizeIterator for PoolIterator<'_, T> {
    fn len(&self) -> usize {
        self.inner.len() - self.garbage_len
    }
}

impl<'pool, T> std::iter::FusedIterator for PoolIterator<'pool, T> where
    std::slice::Iter<'pool, PoolItem<T>>: std::iter::FusedIterator
{
}

/// Iterator for a [`Pool`] that includes element IDs
///
/// Yields only an actual elements.
/// Elements marked as removed are skipped.
#[derive(Clone)]
pub struct PoolElementIterator<'pool, T> {
    inner: Enumerate<std::slice::Iter<'pool, PoolItem<T>>>,
    garbage_len: usize,
}

impl<'pool, T> PoolElementIterator<'pool, T> {
    fn new(pool: &'pool Pool<T>) -> Self {
        PoolElementIterator {
            inner: pool.vec.iter().enumerate(),
            garbage_len: pool.garbage_len(),
        }
    }
}

impl<'pool, T> Iterator for PoolElementIterator<'pool, T> {
    type Item = (ElementId, &'pool T);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let next = self.inner.next()?;
            if !next.1.garbage {
                return Some((ElementId(next.0 as u32), &next.1.item));
            }
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let hint = self.inner.size_hint();
        (
            hint.0.saturating_sub(self.garbage_len),
            hint.1.map(|x| x.saturating_sub(self.garbage_len)),
        )
    }
}

impl<T> DoubleEndedIterator for PoolElementIterator<'_, T> {
    fn next_back(&mut self) -> Option<Self::Item> {
        loop {
            let next = self.inner.next_back()?;
            if !next.1.garbage {
                return Some((ElementId(next.0 as u32), &next.1.item));
            }
        }
    }
}

impl<T> ExactSizeIterator for PoolElementIterator<'_, T> {
    fn len(&self) -> usize {
        self.inner.len() - self.garbage_len
    }
}

impl<'pool, T> std::iter::FusedIterator for PoolElementIterator<'pool, T> where
    std::slice::Iter<'pool, PoolItem<T>>: std::iter::FusedIterator
{
}

/// IntoIterator for a [`Pool`] that includes elements
///
/// Yields only an actual elements.
/// Elements marked as removed are skipped.
#[derive(Clone)]
pub struct PoolIntoIterator<T> {
    inner: std::vec::IntoIter<PoolItem<T>>,
    garbage_len: usize,
}

impl<T> PoolIntoIterator<T> {
    fn new(pool: Pool<T>) -> Self {
        PoolIntoIterator {
            garbage_len: pool.garbage_len(),
            inner: pool.vec.into_iter(),
        }
    }
}

impl<T> Iterator for PoolIntoIterator<T> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let next = self.inner.next()?;
            if !next.garbage {
                return Some(next.item);
            }
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let hint = self.inner.size_hint();
        (
            hint.0.saturating_sub(self.garbage_len),
            hint.1.map(|x| x.saturating_sub(self.garbage_len)),
        )
    }
}

impl<T> DoubleEndedIterator for PoolIntoIterator<T> {
    fn next_back(&mut self) -> Option<Self::Item> {
        loop {
            let next = self.inner.next_back()?;
            if !next.garbage {
                return Some(next.item);
            }
        }
    }
}

impl<T> ExactSizeIterator for PoolIntoIterator<T> {
    fn len(&self) -> usize {
        self.inner.len() - self.garbage_len
    }
}

impl<T> std::iter::FusedIterator for PoolIntoIterator<T> where
    std::vec::IntoIter<PoolItem<T>>: std::iter::FusedIterator
{
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::*;

    struct DummyNotClonableNotSend<'a> {
        pos: TUVec3<u8>,
        special: &'a str,
    }
    impl Position for DummyNotClonableNotSend<'_> {
        type U = u8;

        fn position(&self) -> TUVec3<Self::U> {
            self.pos
        }
    }

    #[test]
    fn test_non_clonable_compile() {
        let mut test_field = "TEST".to_string();

        let mut pool = Pool::<DummyNotClonableNotSend>::default();
        let element = DummyNotClonableNotSend {
            pos: TUVec3::new(1, 2, 3),
            special: &mut test_field,
        };
        let element_id = pool.insert(element);
        let element = &pool[element_id];
        assert_eq!(element.pos, TUVec3::new(1, 2, 3));
    }
}
