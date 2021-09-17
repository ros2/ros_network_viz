# Copyright Unknown???
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
A SortedOrderedSet is a custom MutableSet that sorts and remembers its order.

This means that every entry has an index that can be looked up.  It can also act like a Sequence.

Based on a recipe originally posted to ActiveState Recipes by Raymond Hettiger,
and released under the MIT license.

Modified from the version available at https://github.com/rspeer/ordered-set
to be sorted.
"""
import bisect
from typing import (
    Any,
    Iterable,
    Iterator,
    List,
    MutableSet,
    Optional,
    Sequence,
    TypeVar,
)

SLICE_ALL = slice(None)

T = TypeVar('T')


def _is_atomic(obj: Any) -> bool:
    """
    Return True for iterable objects that should not be iterated whn indexing a SortedOrderedSet.

    When we index by an iterable, usually that means we're being asked to look
    up a list of things.

    However, in the case of the .index() method, we shouldn't handle strings
    and tuples like other iterables. They're not sequences of things to look
    up, they're the single, atomic thing we're trying to find.

    As an example, oset.index('hello') should give the index of 'hello' in an
    SortedOrderedSet of strings. It shouldn't give the indexes of each individual
    character.
    """
    return isinstance(obj, str) or isinstance(obj, tuple)


class SortedOrderedSet(MutableSet[T], Sequence[T]):
    """
    A SortedOrderedSet is a custom MutableSet that sorts and remembers its order.

    This means that that every entry has an index that can be looked up.

    Example:
    -------
        >>> SortedOrderedSet([1, 1, 2, 3, 2])
        SortedOrderedSet([1, 2, 3])

    """

    def __init__(self, iterable: Optional[Iterable[T]] = None):
        self.items = []
        self.map = {}
        if iterable is not None:
            self |= iterable

    def __len__(self):
        """
        Return the number of unique elements in the ordered set.

        Example:
        -------
            >>> len(SortedOrderedSet([]))
            0
            >>> len(SortedOrderedSet([1, 2]))
            2

        """
        return len(self.items)

    def __getitem__(self, index: int) -> T:
        """
        Get the item at a given index.

        If `index` is a slice, you will get back that slice of items, as a
        new SortedOrderedSet.

        If `index` is a list or a similar iterable, you'll get a list of
        items corresponding to those indices. This is similar to NumPy's
        "fancy indexing". The result is not an SortedOrderedSet because you may ask
        for duplicate indices, and the number of elements returned should be
        the number of elements asked for.

        Example:
        -------
            >>> oset = SortedOrderedSet([1, 2, 3])
            >>> oset[1]
            2

        """
        if isinstance(index, slice) and index == SLICE_ALL:
            return self.copy()
        elif isinstance(index, Iterable):
            return [self.items[i] for i in index]
        elif isinstance(index, slice) or hasattr(index, '__index__'):
            result = self.items[index]
            if isinstance(result, list):
                return self.__class__(result)
            else:
                return result
        else:
            raise TypeError("Don't know how to index an SortedOrderedSet by %r" % index)

    def copy(self) -> 'SortedOrderedSet[T]':
        """
        Return a shallow copy of this object.

        Example:
        -------
            >>> this = SortedOrderedSet([1, 2, 3])
            >>> other = this.copy()
            >>> this == other
            True
            >>> this is other
            False

        """
        return self.__class__(self)

    def __contains__(self, key: Any) -> bool:
        """
        Test if the item is in this ordered set.

        Example:
        -------
            >>> 1 in SortedOrderedSet([1, 3, 2])
            True
            >>> 5 in SortedOrderedSet([1, 3, 2])
            False

        """
        return key in self.map

    def add(self, key: T) -> int:
        """
        Add `key` as an item to this SortedOrderedSet, then return its index.

        If `key` is already in the SortedOrderedSet, return the index it already
        had.

        Example:
        -------
            >>> oset = SortedOrderedSet()
            >>> oset.append(3)
            0
            >>> print(oset)
            SortedOrderedSet([3])

        """
        if key not in self.map:
            index = bisect.bisect_left(self.items, key)
            self.items.insert(index, key)
            self.map[key] = index
            for item in self.items[(index + 1):]:
                self.map[item] = self.map[item] + 1

        return self.map[key]

    def index(self, key: Sequence[T]) -> List[int]:
        """
        Get the index of a given entry, raising an IndexError if it is not present.

        `key` can be an iterable of entries that is not a string, in which case
        this returns a list of indices.

        Example:
        -------
            >>> oset = SortedOrderedSet([1, 2, 3])
            >>> oset.index(2)
            1

        """
        if isinstance(key, Iterable) and not _is_atomic(key):
            return [self.index(subkey) for subkey in key]
        return self.map[key]

    def discard(self, key: T) -> None:
        """
        Remove an element; do not raise an exception if absent.

        The MutableSet mixin uses this to implement the .remove() method, which
        *does* raise an error when asked to remove a non-existent item.

        Example:
        -------
            >>> oset = SortedOrderedSet([1, 2, 3])
            >>> oset.discard(2)
            >>> print(oset)
            SortedOrderedSet([1, 3])
            >>> oset.discard(2)
            >>> print(oset)
            SortedOrderedSet([1, 3])

        """
        if key in self:
            index = self.map[key]
            for item in self.items[(index + 1):]:
                self.map[item] = self.map[item] - 1
            del self.items[index]
            del self.map[key]

    def __iter__(self) -> Iterator[T]:
        """
        Iterate over the elements in the set.

        Example:
        -------
            >>> list(iter(SortedOrderedSet([1, 2, 3])))
            [1, 2, 3]

        """
        return iter(self.items)

    def __repr__(self) -> str:
        if not self:
            return '%s()' % (self.__class__.__name__,)
        return '%s(%r)' % (self.__class__.__name__, list(self))
