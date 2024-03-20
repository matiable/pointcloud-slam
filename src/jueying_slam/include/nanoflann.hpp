

#ifndef NANOFLANN_HPP_
#define NANOFLANN_HPP_

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>   
#include <cstdio>  
#include <cstdlib> 
#include <functional>
#include <limits> 
#include <stdexcept>
#include <vector>

#define NANOFLANN_VERSION 0x132

#if !defined(NOMINMAX) &&                                                      \
    (defined(_WIN32) || defined(_WIN32_) || defined(WIN32) || defined(_WIN64))
#define NOMINMAX
#ifdef max
#undef max
#undef min
#endif
#endif

namespace nanoflann {

template <typename T> T pi_const() {
  return static_cast<T>(3.14159265358979323846);
}

template <typename T, typename = int> struct has_resize : std::false_type {};

template <typename T>
struct has_resize<T, decltype((void)std::declval<T>().resize(1), 0)>
    : std::true_type {};

template <typename T, typename = int> struct has_assign : std::false_type {};

template <typename T>
struct has_assign<T, decltype((void)std::declval<T>().assign(1, 0), 0)>
    : std::true_type {};

template <typename Container>
inline typename std::enable_if<has_resize<Container>::value, void>::type
resize(Container &c, const size_t nElements) {
  c.resize(nElements);
}

template <typename Container>
inline typename std::enable_if<!has_resize<Container>::value, void>::type
resize(Container &c, const size_t nElements) {
  if (nElements != c.size())
    throw std::logic_error("Try to change the size of a std::array.");
}

template <typename Container, typename T>
inline typename std::enable_if<has_assign<Container>::value, void>::type
assign(Container &c, const size_t nElements, const T &value) {
  c.assign(nElements, value);
}

template <typename Container, typename T>
inline typename std::enable_if<!has_assign<Container>::value, void>::type
assign(Container &c, const size_t nElements, const T &value) {
  for (size_t i = 0; i < nElements; i++)
    c[i] = value;
}

template <typename _DistanceType, typename _IndexType = size_t,
          typename _CountType = size_t>
class KNNResultSet {
public:
  typedef _DistanceType DistanceType;
  typedef _IndexType IndexType;
  typedef _CountType CountType;

private:
  IndexType *indices;
  DistanceType *dists;
  CountType capacity;
  CountType count;

public:
  inline KNNResultSet(CountType capacity_)
      : indices(0), dists(0), capacity(capacity_), count(0) {}

  inline void init(IndexType *indices_, DistanceType *dists_) {
    indices = indices_;
    dists = dists_;
    count = 0;
    if (capacity)
      dists[capacity - 1] = (std::numeric_limits<DistanceType>::max)();
  }

  inline CountType size() const { return count; }

  inline bool full() const { return count == capacity; }

  inline bool addPoint(DistanceType dist, IndexType index) {
    CountType i;
    for (i = count; i > 0; --i) {
#ifdef NANOFLANN_FIRST_MATCH 

      if ((dists[i - 1] > dist) ||
          ((dist == dists[i - 1]) && (indices[i - 1] > index))) {
#else
      if (dists[i - 1] > dist) {
#endif
        if (i < capacity) {
          dists[i] = dists[i - 1];
          indices[i] = indices[i - 1];
        }
      } else
        break;
    }
    if (i < capacity) {
      dists[i] = dist;
      indices[i] = index;
    }
    if (count < capacity)
      count++;

    return true;
  }

  inline DistanceType worstDist() const { return dists[capacity - 1]; }
};

struct IndexDist_Sorter {

  template <typename PairType>
  inline bool operator()(const PairType &p1, const PairType &p2) const {
    return p1.second < p2.second;
  }
};

template <typename _DistanceType, typename _IndexType = size_t>
class RadiusResultSet {
public:
  typedef _DistanceType DistanceType;
  typedef _IndexType IndexType;

public:
  const DistanceType radius;

  std::vector<std::pair<IndexType, DistanceType>> &m_indices_dists;

  inline RadiusResultSet(
      DistanceType radius_,
      std::vector<std::pair<IndexType, DistanceType>> &indices_dists)
      : radius(radius_), m_indices_dists(indices_dists) {
    init();
  }

  inline void init() { clear(); }
  inline void clear() { m_indices_dists.clear(); }

  inline size_t size() const { return m_indices_dists.size(); }

  inline bool full() const { return true; }

  inline bool addPoint(DistanceType dist, IndexType index) {
    if (dist < radius)
      m_indices_dists.push_back(std::make_pair(index, dist));
    return true;
  }

  inline DistanceType worstDist() const { return radius; }

  std::pair<IndexType, DistanceType> worst_item() const {
    if (m_indices_dists.empty())
      throw std::runtime_error("Cannot invoke RadiusResultSet::worst_item() on "
                               "an empty list of results.");
    typedef
        typename std::vector<std::pair<IndexType, DistanceType>>::const_iterator
            DistIt;
    DistIt it = std::max_element(m_indices_dists.begin(), m_indices_dists.end(),
                                 IndexDist_Sorter());
    return *it;
  }
};

template <typename T>
void save_value(FILE *stream, const T &value, size_t count = 1) {
  fwrite(&value, sizeof(value), count, stream);
}

template <typename T>
void save_value(FILE *stream, const std::vector<T> &value) {
  size_t size = value.size();
  fwrite(&size, sizeof(size_t), 1, stream);
  fwrite(&value[0], sizeof(T), size, stream);
}

template <typename T>
void load_value(FILE *stream, T &value, size_t count = 1) {
  size_t read_cnt = fread(&value, sizeof(value), count, stream);
  if (read_cnt != count) {
    throw std::runtime_error("Cannot read from file");
  }
}

template <typename T> void load_value(FILE *stream, std::vector<T> &value) {
  size_t size;
  size_t read_cnt = fread(&size, sizeof(size_t), 1, stream);
  if (read_cnt != 1) {
    throw std::runtime_error("Cannot read from file");
  }
  value.resize(size);
  read_cnt = fread(&value[0], sizeof(T), size, stream);
  if (read_cnt != size) {
    throw std::runtime_error("Cannot read from file");
  }
}

struct Metric {};

template <class T, class DataSource, typename _DistanceType = T>
struct L1_Adaptor {
  typedef T ElementType;
  typedef _DistanceType DistanceType;

  const DataSource &data_source;

  L1_Adaptor(const DataSource &_data_source) : data_source(_data_source) {}

  inline DistanceType evalMetric(const T *a, const size_t b_idx, size_t size,
                                 DistanceType worst_dist = -1) const {
    DistanceType result = DistanceType();
    const T *last = a + size;
    const T *lastgroup = last - 3;
    size_t d = 0;

    while (a < lastgroup) {
      const DistanceType diff0 =
          std::abs(a[0] - data_source.kdtree_get_pt(b_idx, d++));
      const DistanceType diff1 =
          std::abs(a[1] - data_source.kdtree_get_pt(b_idx, d++));
      const DistanceType diff2 =
          std::abs(a[2] - data_source.kdtree_get_pt(b_idx, d++));
      const DistanceType diff3 =
          std::abs(a[3] - data_source.kdtree_get_pt(b_idx, d++));
      result += diff0 + diff1 + diff2 + diff3;
      a += 4;
      if ((worst_dist > 0) && (result > worst_dist)) {
        return result;
      }
    }

    while (a < last) {
      result += std::abs(*a++ - data_source.kdtree_get_pt(b_idx, d++));
    }
    return result;
  }

  template <typename U, typename V>
  inline DistanceType accum_dist(const U a, const V b, const size_t) const {
    return std::abs(a - b);
  }
};

template <class T, class DataSource, typename _DistanceType = T>
struct L2_Adaptor {
  typedef T ElementType;
  typedef _DistanceType DistanceType;

  const DataSource &data_source;

  L2_Adaptor(const DataSource &_data_source) : data_source(_data_source) {}

  inline DistanceType evalMetric(const T *a, const size_t b_idx, size_t size,
                                 DistanceType worst_dist = -1) const {
    DistanceType result = DistanceType();
    const T *last = a + size;
    const T *lastgroup = last - 3;
    size_t d = 0;

    while (a < lastgroup) {
      const DistanceType diff0 = a[0] - data_source.kdtree_get_pt(b_idx, d++);
      const DistanceType diff1 = a[1] - data_source.kdtree_get_pt(b_idx, d++);
      const DistanceType diff2 = a[2] - data_source.kdtree_get_pt(b_idx, d++);
      const DistanceType diff3 = a[3] - data_source.kdtree_get_pt(b_idx, d++);
      result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
      a += 4;
      if ((worst_dist > 0) && (result > worst_dist)) {
        return result;
      }
    }

    while (a < last) {
      const DistanceType diff0 = *a++ - data_source.kdtree_get_pt(b_idx, d++);
      result += diff0 * diff0;
    }
    return result;
  }

  template <typename U, typename V>
  inline DistanceType accum_dist(const U a, const V b, const size_t) const {
    return (a - b) * (a - b);
  }
};

template <class T, class DataSource, typename _DistanceType = T>
struct L2_Simple_Adaptor {
  typedef T ElementType;
  typedef _DistanceType DistanceType;

  const DataSource &data_source;

  L2_Simple_Adaptor(const DataSource &_data_source)
      : data_source(_data_source) {}

  inline DistanceType evalMetric(const T *a, const size_t b_idx,
                                 size_t size) const {
    DistanceType result = DistanceType();
    for (size_t i = 0; i < size; ++i) {
      const DistanceType diff = a[i] - data_source.kdtree_get_pt(b_idx, i);
      result += diff * diff;
    }
    return result;
  }

  template <typename U, typename V>
  inline DistanceType accum_dist(const U a, const V b, const size_t) const {
    return (a - b) * (a - b);
  }
};

template <class T, class DataSource, typename _DistanceType = T>
struct SO2_Adaptor {
  typedef T ElementType;
  typedef _DistanceType DistanceType;

  const DataSource &data_source;

  SO2_Adaptor(const DataSource &_data_source) : data_source(_data_source) {}

  inline DistanceType evalMetric(const T *a, const size_t b_idx,
                                 size_t size) const {
    return accum_dist(a[size - 1], data_source.kdtree_get_pt(b_idx, size - 1),
                      size - 1);
  }

  template <typename U, typename V>
  inline DistanceType accum_dist(const U a, const V b, const size_t) const {
    DistanceType result = DistanceType();
    DistanceType PI = pi_const<DistanceType>();
    result = b - a;
    if (result > PI)
      result -= 2 * PI;
    else if (result < -PI)
      result += 2 * PI;
    return result;
  }
};

template <class T, class DataSource, typename _DistanceType = T>
struct SO3_Adaptor {
  typedef T ElementType;
  typedef _DistanceType DistanceType;

  L2_Simple_Adaptor<T, DataSource> distance_L2_Simple;

  SO3_Adaptor(const DataSource &_data_source)
      : distance_L2_Simple(_data_source) {}

  inline DistanceType evalMetric(const T *a, const size_t b_idx,
                                 size_t size) const {
    return distance_L2_Simple.evalMetric(a, b_idx, size);
  }

  template <typename U, typename V>
  inline DistanceType accum_dist(const U a, const V b, const size_t idx) const {
    return distance_L2_Simple.accum_dist(a, b, idx);
  }
};

struct metric_L1 : public Metric {
  template <class T, class DataSource> struct traits {
    typedef L1_Adaptor<T, DataSource> distance_t;
  };
};

struct metric_L2 : public Metric {
  template <class T, class DataSource> struct traits {
    typedef L2_Adaptor<T, DataSource> distance_t;
  };
};

struct metric_L2_Simple : public Metric {
  template <class T, class DataSource> struct traits {
    typedef L2_Simple_Adaptor<T, DataSource> distance_t;
  };
};

struct metric_SO2 : public Metric {
  template <class T, class DataSource> struct traits {
    typedef SO2_Adaptor<T, DataSource> distance_t;
  };
};

struct metric_SO3 : public Metric {
  template <class T, class DataSource> struct traits {
    typedef SO3_Adaptor<T, DataSource> distance_t;
  };
};

struct KDTreeSingleIndexAdaptorParams {
  KDTreeSingleIndexAdaptorParams(size_t _leaf_max_size = 10)
      : leaf_max_size(_leaf_max_size) {}

  size_t leaf_max_size;
};

struct SearchParams {

  SearchParams(int checks_IGNORED_ = 32, float eps_ = 0, bool sorted_ = true)
      : checks(checks_IGNORED_), eps(eps_), sorted(sorted_) {}

  int checks;  

  float eps;   
  bool sorted; 

};

template <typename T> inline T *allocate(size_t count = 1) {
  T *mem = static_cast<T *>(::malloc(sizeof(T) * count));
  return mem;
}

const size_t WORDSIZE = 16;
const size_t BLOCKSIZE = 8192;

class PooledAllocator {

  size_t remaining; 
  void *base;       
  void *loc;        

  void internal_init() {
    remaining = 0;
    base = NULL;
    usedMemory = 0;
    wastedMemory = 0;
  }

public:
  size_t usedMemory;
  size_t wastedMemory;

  PooledAllocator() { internal_init(); }

  ~PooledAllocator() { free_all(); }

  void free_all() {
    while (base != NULL) {
      void *prev =
          *(static_cast<void **>(base)); 
      ::free(base);
      base = prev;
    }
    internal_init();
  }

  void *malloc(const size_t req_size) {

    const size_t size = (req_size + (WORDSIZE - 1)) & ~(WORDSIZE - 1);

    if (size > remaining) {

      wastedMemory += remaining;

      const size_t blocksize =
          (size + sizeof(void *) + (WORDSIZE - 1) > BLOCKSIZE)
              ? size + sizeof(void *) + (WORDSIZE - 1)
              : BLOCKSIZE;

      void *m = ::malloc(blocksize);
      if (!m) {
        fprintf(stderr, "Failed to allocate memory.\n");
        return NULL;
      }

      static_cast<void **>(m)[0] = base;
      base = m;

      size_t shift = 0;

      remaining = blocksize - sizeof(void *) - shift;
      loc = (static_cast<char *>(m) + sizeof(void *) + shift);
    }
    void *rloc = loc;
    loc = static_cast<char *>(loc) + size;
    remaining -= size;

    usedMemory += size;

    return rloc;
  }

  template <typename T> T *allocate(const size_t count = 1) {
    T *mem = static_cast<T *>(this->malloc(sizeof(T) * count));
    return mem;
  }
};

template <int DIM, typename T> struct array_or_vector_selector {
  typedef std::array<T, DIM> container_t;
};

template <typename T> struct array_or_vector_selector<-1, T> {
  typedef std::vector<T> container_t;
};

template <class Derived, typename Distance, class DatasetAdaptor, int DIM = -1,
          typename IndexType = size_t>
class KDTreeBaseClass {

public:

  void freeIndex(Derived &obj) {
    obj.pool.free_all();
    obj.root_node = NULL;
    obj.m_size_at_index_build = 0;
  }

  typedef typename Distance::ElementType ElementType;
  typedef typename Distance::DistanceType DistanceType;

  struct Node {

    union {
      struct leaf {
        IndexType left, right; 
      } lr;
      struct nonleaf {
        int divfeat;                  
        DistanceType divlow, divhigh; 
      } sub;
    } node_type;
    Node *child1, *child2; 
  };

  typedef Node *NodePtr;

  struct Interval {
    ElementType low, high;
  };

  std::vector<IndexType> vind;

  NodePtr root_node;

  size_t m_leaf_max_size;

  size_t m_size;                
  size_t m_size_at_index_build; 

  int dim;                      

  typedef
      typename array_or_vector_selector<DIM, Interval>::container_t BoundingBox;

  typedef typename array_or_vector_selector<DIM, DistanceType>::container_t
      distance_vector_t;

  BoundingBox root_bbox;

  PooledAllocator pool;

  size_t size(const Derived &obj) const { return obj.m_size; }

  size_t veclen(const Derived &obj) {
    return static_cast<size_t>(DIM > 0 ? DIM : obj.dim);
  }

  inline ElementType dataset_get(const Derived &obj, size_t idx,
                                 int component) const {
    return obj.dataset.kdtree_get_pt(idx, component);
  }

  size_t usedMemory(Derived &obj) {
    return obj.pool.usedMemory + obj.pool.wastedMemory +
           obj.dataset.kdtree_get_point_count() *
               sizeof(IndexType); 
  }

  void computeMinMax(const Derived &obj, IndexType *ind, IndexType count,
                     int element, ElementType &min_elem,
                     ElementType &max_elem) {
    min_elem = dataset_get(obj, ind[0], element);
    max_elem = dataset_get(obj, ind[0], element);
    for (IndexType i = 1; i < count; ++i) {
      ElementType val = dataset_get(obj, ind[i], element);
      if (val < min_elem)
        min_elem = val;
      if (val > max_elem)
        max_elem = val;
    }
  }

  NodePtr divideTree(Derived &obj, const IndexType left, const IndexType right,
                     BoundingBox &bbox) {
    NodePtr node = obj.pool.template allocate<Node>(); 

    if ((right - left) <= static_cast<IndexType>(obj.m_leaf_max_size)) {
      node->child1 = node->child2 = NULL; 
      node->node_type.lr.left = left;
      node->node_type.lr.right = right;

      for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i) {
        bbox[i].low = dataset_get(obj, obj.vind[left], i);
        bbox[i].high = dataset_get(obj, obj.vind[left], i);
      }
      for (IndexType k = left + 1; k < right; ++k) {
        for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i) {
          if (bbox[i].low > dataset_get(obj, obj.vind[k], i))
            bbox[i].low = dataset_get(obj, obj.vind[k], i);
          if (bbox[i].high < dataset_get(obj, obj.vind[k], i))
            bbox[i].high = dataset_get(obj, obj.vind[k], i);
        }
      }
    } else {
      IndexType idx;
      int cutfeat;
      DistanceType cutval;
      middleSplit_(obj, &obj.vind[0] + left, right - left, idx, cutfeat, cutval,
                   bbox);

      node->node_type.sub.divfeat = cutfeat;

      BoundingBox left_bbox(bbox);
      left_bbox[cutfeat].high = cutval;
      node->child1 = divideTree(obj, left, left + idx, left_bbox);

      BoundingBox right_bbox(bbox);
      right_bbox[cutfeat].low = cutval;
      node->child2 = divideTree(obj, left + idx, right, right_bbox);

      node->node_type.sub.divlow = left_bbox[cutfeat].high;
      node->node_type.sub.divhigh = right_bbox[cutfeat].low;

      for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i) {
        bbox[i].low = std::min(left_bbox[i].low, right_bbox[i].low);
        bbox[i].high = std::max(left_bbox[i].high, right_bbox[i].high);
      }
    }

    return node;
  }

  void middleSplit_(Derived &obj, IndexType *ind, IndexType count,
                    IndexType &index, int &cutfeat, DistanceType &cutval,
                    const BoundingBox &bbox) {
    const DistanceType EPS = static_cast<DistanceType>(0.00001);
    ElementType max_span = bbox[0].high - bbox[0].low;
    for (int i = 1; i < (DIM > 0 ? DIM : obj.dim); ++i) {
      ElementType span = bbox[i].high - bbox[i].low;
      if (span > max_span) {
        max_span = span;
      }
    }
    ElementType max_spread = -1;
    cutfeat = 0;
    for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i) {
      ElementType span = bbox[i].high - bbox[i].low;
      if (span > (1 - EPS) * max_span) {
        ElementType min_elem, max_elem;
        computeMinMax(obj, ind, count, i, min_elem, max_elem);
        ElementType spread = max_elem - min_elem;
        ;
        if (spread > max_spread) {
          cutfeat = i;
          max_spread = spread;
        }
      }
    }

    DistanceType split_val = (bbox[cutfeat].low + bbox[cutfeat].high) / 2;
    ElementType min_elem, max_elem;
    computeMinMax(obj, ind, count, cutfeat, min_elem, max_elem);

    if (split_val < min_elem)
      cutval = min_elem;
    else if (split_val > max_elem)
      cutval = max_elem;
    else
      cutval = split_val;

    IndexType lim1, lim2;
    planeSplit(obj, ind, count, cutfeat, cutval, lim1, lim2);

    if (lim1 > count / 2)
      index = lim1;
    else if (lim2 < count / 2)
      index = lim2;
    else
      index = count / 2;
  }

  void planeSplit(Derived &obj, IndexType *ind, const IndexType count,
                  int cutfeat, DistanceType &cutval, IndexType &lim1,
                  IndexType &lim2) {

    IndexType left = 0;
    IndexType right = count - 1;
    for (;;) {
      while (left <= right && dataset_get(obj, ind[left], cutfeat) < cutval)
        ++left;
      while (right && left <= right &&
             dataset_get(obj, ind[right], cutfeat) >= cutval)
        --right;
      if (left > right || !right)
        break; 
      std::swap(ind[left], ind[right]);
      ++left;
      --right;
    }

    lim1 = left;
    right = count - 1;
    for (;;) {
      while (left <= right && dataset_get(obj, ind[left], cutfeat) <= cutval)
        ++left;
      while (right && left <= right &&
             dataset_get(obj, ind[right], cutfeat) > cutval)
        --right;
      if (left > right || !right)
        break; 
      std::swap(ind[left], ind[right]);
      ++left;
      --right;
    }
    lim2 = left;
  }

  DistanceType computeInitialDistances(const Derived &obj,
                                       const ElementType *vec,
                                       distance_vector_t &dists) const {
    assert(vec);
    DistanceType distsq = DistanceType();

    for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i) {
      if (vec[i] < obj.root_bbox[i].low) {
        dists[i] = obj.distance.accum_dist(vec[i], obj.root_bbox[i].low, i);
        distsq += dists[i];
      }
      if (vec[i] > obj.root_bbox[i].high) {
        dists[i] = obj.distance.accum_dist(vec[i], obj.root_bbox[i].high, i);
        distsq += dists[i];
      }
    }
    return distsq;
  }

  void save_tree(Derived &obj, FILE *stream, NodePtr tree) {
    save_value(stream, *tree);
    if (tree->child1 != NULL) {
      save_tree(obj, stream, tree->child1);
    }
    if (tree->child2 != NULL) {
      save_tree(obj, stream, tree->child2);
    }
  }

  void load_tree(Derived &obj, FILE *stream, NodePtr &tree) {
    tree = obj.pool.template allocate<Node>();
    load_value(stream, *tree);
    if (tree->child1 != NULL) {
      load_tree(obj, stream, tree->child1);
    }
    if (tree->child2 != NULL) {
      load_tree(obj, stream, tree->child2);
    }
  }

  void saveIndex_(Derived &obj, FILE *stream) {
    save_value(stream, obj.m_size);
    save_value(stream, obj.dim);
    save_value(stream, obj.root_bbox);
    save_value(stream, obj.m_leaf_max_size);
    save_value(stream, obj.vind);
    save_tree(obj, stream, obj.root_node);
  }

  void loadIndex_(Derived &obj, FILE *stream) {
    load_value(stream, obj.m_size);
    load_value(stream, obj.dim);
    load_value(stream, obj.root_bbox);
    load_value(stream, obj.m_leaf_max_size);
    load_value(stream, obj.vind);
    load_tree(obj, stream, obj.root_node);
  }
};

template <typename Distance, class DatasetAdaptor, int DIM = -1,
          typename IndexType = size_t>
class KDTreeSingleIndexAdaptor
    : public KDTreeBaseClass<
          KDTreeSingleIndexAdaptor<Distance, DatasetAdaptor, DIM, IndexType>,
          Distance, DatasetAdaptor, DIM, IndexType> {
public:

  KDTreeSingleIndexAdaptor(
      const KDTreeSingleIndexAdaptor<Distance, DatasetAdaptor, DIM, IndexType>
          &) = delete;

  const DatasetAdaptor &dataset; 

  const KDTreeSingleIndexAdaptorParams index_params;

  Distance distance;

  typedef typename nanoflann::KDTreeBaseClass<
      nanoflann::KDTreeSingleIndexAdaptor<Distance, DatasetAdaptor, DIM,
                                          IndexType>,
      Distance, DatasetAdaptor, DIM, IndexType>
      BaseClassRef;

  typedef typename BaseClassRef::ElementType ElementType;
  typedef typename BaseClassRef::DistanceType DistanceType;

  typedef typename BaseClassRef::Node Node;
  typedef Node *NodePtr;

  typedef typename BaseClassRef::Interval Interval;

  typedef typename BaseClassRef::BoundingBox BoundingBox;

  typedef typename BaseClassRef::distance_vector_t distance_vector_t;

  KDTreeSingleIndexAdaptor(const int dimensionality,
                           const DatasetAdaptor &inputData,
                           const KDTreeSingleIndexAdaptorParams &params =
                               KDTreeSingleIndexAdaptorParams())
      : dataset(inputData), index_params(params), distance(inputData) {
    BaseClassRef::root_node = NULL;
    BaseClassRef::m_size = dataset.kdtree_get_point_count();
    BaseClassRef::m_size_at_index_build = BaseClassRef::m_size;
    BaseClassRef::dim = dimensionality;
    if (DIM > 0)
      BaseClassRef::dim = DIM;
    BaseClassRef::m_leaf_max_size = params.leaf_max_size;

    init_vind();
  }

  void buildIndex() {
    BaseClassRef::m_size = dataset.kdtree_get_point_count();
    BaseClassRef::m_size_at_index_build = BaseClassRef::m_size;
    init_vind();
    this->freeIndex(*this);
    BaseClassRef::m_size_at_index_build = BaseClassRef::m_size;
    if (BaseClassRef::m_size == 0)
      return;
    computeBoundingBox(BaseClassRef::root_bbox);
    BaseClassRef::root_node =
        this->divideTree(*this, 0, BaseClassRef::m_size,
                         BaseClassRef::root_bbox); 
  }

  template <typename RESULTSET>
  bool findNeighbors(RESULTSET &result, const ElementType *vec,
                     const SearchParams &searchParams) const {
    assert(vec);
    if (this->size(*this) == 0)
      return false;
    if (!BaseClassRef::root_node)
      throw std::runtime_error(
          "[nanoflann] findNeighbors() called before building the index.");
    float epsError = 1 + searchParams.eps;

    distance_vector_t
        dists; 
    auto zero = static_cast<decltype(result.worstDist())>(0);
    assign(dists, (DIM > 0 ? DIM : BaseClassRef::dim),
           zero); 
    DistanceType distsq = this->computeInitialDistances(*this, vec, dists);
    searchLevel(result, vec, BaseClassRef::root_node, distsq, dists,
                epsError); 

    return result.full();
  }

  size_t knnSearch(const ElementType *query_point, const size_t num_closest,
                   IndexType *out_indices, DistanceType *out_distances_sq,
                   const int  = 10) const {
    nanoflann::KNNResultSet<DistanceType, IndexType> resultSet(num_closest);
    resultSet.init(out_indices, out_distances_sq);
    this->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
    return resultSet.size();
  }

  size_t
  radiusSearch(const ElementType *query_point, const DistanceType &radius,
               std::vector<std::pair<IndexType, DistanceType>> &IndicesDists,
               const SearchParams &searchParams) const {
    RadiusResultSet<DistanceType, IndexType> resultSet(radius, IndicesDists);
    const size_t nFound =
        radiusSearchCustomCallback(query_point, resultSet, searchParams);
    if (searchParams.sorted)
      std::sort(IndicesDists.begin(), IndicesDists.end(), IndexDist_Sorter());
    return nFound;
  }

  template <class SEARCH_CALLBACK>
  size_t radiusSearchCustomCallback(
      const ElementType *query_point, SEARCH_CALLBACK &resultSet,
      const SearchParams &searchParams = SearchParams()) const {
    this->findNeighbors(resultSet, query_point, searchParams);
    return resultSet.size();
  }

public:

  void init_vind() {

    BaseClassRef::m_size = dataset.kdtree_get_point_count();
    if (BaseClassRef::vind.size() != BaseClassRef::m_size)
      BaseClassRef::vind.resize(BaseClassRef::m_size);
    for (size_t i = 0; i < BaseClassRef::m_size; i++)
      BaseClassRef::vind[i] = i;
  }

  void computeBoundingBox(BoundingBox &bbox) {
    resize(bbox, (DIM > 0 ? DIM : BaseClassRef::dim));
    if (dataset.kdtree_get_bbox(bbox)) {

    } else {
      const size_t N = dataset.kdtree_get_point_count();
      if (!N)
        throw std::runtime_error("[nanoflann] computeBoundingBox() called but "
                                 "no data points found.");
      for (int i = 0; i < (DIM > 0 ? DIM : BaseClassRef::dim); ++i) {
        bbox[i].low = bbox[i].high = this->dataset_get(*this, 0, i);
      }
      for (size_t k = 1; k < N; ++k) {
        for (int i = 0; i < (DIM > 0 ? DIM : BaseClassRef::dim); ++i) {
          if (this->dataset_get(*this, k, i) < bbox[i].low)
            bbox[i].low = this->dataset_get(*this, k, i);
          if (this->dataset_get(*this, k, i) > bbox[i].high)
            bbox[i].high = this->dataset_get(*this, k, i);
        }
      }
    }
  }

  template <class RESULTSET>
  bool searchLevel(RESULTSET &result_set, const ElementType *vec,
                   const NodePtr node, DistanceType mindistsq,
                   distance_vector_t &dists, const float epsError) const {

    if ((node->child1 == NULL) && (node->child2 == NULL)) {

      DistanceType worst_dist = result_set.worstDist();
      for (IndexType i = node->node_type.lr.left; i < node->node_type.lr.right;
           ++i) {
        const IndexType index = BaseClassRef::vind[i]; 
        DistanceType dist = distance.evalMetric(
            vec, index, (DIM > 0 ? DIM : BaseClassRef::dim));
        if (dist < worst_dist) {
          if (!result_set.addPoint(dist, BaseClassRef::vind[i])) {

            return false;
          }
        }
      }
      return true;
    }

    int idx = node->node_type.sub.divfeat;
    ElementType val = vec[idx];
    DistanceType diff1 = val - node->node_type.sub.divlow;
    DistanceType diff2 = val - node->node_type.sub.divhigh;

    NodePtr bestChild;
    NodePtr otherChild;
    DistanceType cut_dist;
    if ((diff1 + diff2) < 0) {
      bestChild = node->child1;
      otherChild = node->child2;
      cut_dist = distance.accum_dist(val, node->node_type.sub.divhigh, idx);
    } else {
      bestChild = node->child2;
      otherChild = node->child1;
      cut_dist = distance.accum_dist(val, node->node_type.sub.divlow, idx);
    }

    if (!searchLevel(result_set, vec, bestChild, mindistsq, dists, epsError)) {

      return false;
    }

    DistanceType dst = dists[idx];
    mindistsq = mindistsq + cut_dist - dst;
    dists[idx] = cut_dist;
    if (mindistsq * epsError <= result_set.worstDist()) {
      if (!searchLevel(result_set, vec, otherChild, mindistsq, dists,
                       epsError)) {

        return false;
      }
    }
    dists[idx] = dst;
    return true;
  }

public:

  void saveIndex(FILE *stream) { this->saveIndex_(*this, stream); }

  void loadIndex(FILE *stream) { this->loadIndex_(*this, stream); }

}; 

template <typename Distance, class DatasetAdaptor, int DIM = -1,
          typename IndexType = size_t>
class KDTreeSingleIndexDynamicAdaptor_
    : public KDTreeBaseClass<KDTreeSingleIndexDynamicAdaptor_<
                                 Distance, DatasetAdaptor, DIM, IndexType>,
                             Distance, DatasetAdaptor, DIM, IndexType> {
public:

  const DatasetAdaptor &dataset; 

  KDTreeSingleIndexAdaptorParams index_params;

  std::vector<int> &treeIndex;

  Distance distance;

  typedef typename nanoflann::KDTreeBaseClass<
      nanoflann::KDTreeSingleIndexDynamicAdaptor_<Distance, DatasetAdaptor, DIM,
                                                  IndexType>,
      Distance, DatasetAdaptor, DIM, IndexType>
      BaseClassRef;

  typedef typename BaseClassRef::ElementType ElementType;
  typedef typename BaseClassRef::DistanceType DistanceType;

  typedef typename BaseClassRef::Node Node;
  typedef Node *NodePtr;

  typedef typename BaseClassRef::Interval Interval;

  typedef typename BaseClassRef::BoundingBox BoundingBox;

  typedef typename BaseClassRef::distance_vector_t distance_vector_t;

  KDTreeSingleIndexDynamicAdaptor_(
      const int dimensionality, const DatasetAdaptor &inputData,
      std::vector<int> &treeIndex_,
      const KDTreeSingleIndexAdaptorParams &params =
          KDTreeSingleIndexAdaptorParams())
      : dataset(inputData), index_params(params), treeIndex(treeIndex_),
        distance(inputData) {
    BaseClassRef::root_node = NULL;
    BaseClassRef::m_size = 0;
    BaseClassRef::m_size_at_index_build = 0;
    BaseClassRef::dim = dimensionality;
    if (DIM > 0)
      BaseClassRef::dim = DIM;
    BaseClassRef::m_leaf_max_size = params.leaf_max_size;
  }

  KDTreeSingleIndexDynamicAdaptor_
  operator=(const KDTreeSingleIndexDynamicAdaptor_ &rhs) {
    KDTreeSingleIndexDynamicAdaptor_ tmp(rhs);
    std::swap(BaseClassRef::vind, tmp.BaseClassRef::vind);
    std::swap(BaseClassRef::m_leaf_max_size, tmp.BaseClassRef::m_leaf_max_size);
    std::swap(index_params, tmp.index_params);
    std::swap(treeIndex, tmp.treeIndex);
    std::swap(BaseClassRef::m_size, tmp.BaseClassRef::m_size);
    std::swap(BaseClassRef::m_size_at_index_build,
              tmp.BaseClassRef::m_size_at_index_build);
    std::swap(BaseClassRef::root_node, tmp.BaseClassRef::root_node);
    std::swap(BaseClassRef::root_bbox, tmp.BaseClassRef::root_bbox);
    std::swap(BaseClassRef::pool, tmp.BaseClassRef::pool);
    return *this;
  }

  void buildIndex() {
    BaseClassRef::m_size = BaseClassRef::vind.size();
    this->freeIndex(*this);
    BaseClassRef::m_size_at_index_build = BaseClassRef::m_size;
    if (BaseClassRef::m_size == 0)
      return;
    computeBoundingBox(BaseClassRef::root_bbox);
    BaseClassRef::root_node =
        this->divideTree(*this, 0, BaseClassRef::m_size,
                         BaseClassRef::root_bbox); 
  }

  template <typename RESULTSET>
  bool findNeighbors(RESULTSET &result, const ElementType *vec,
                     const SearchParams &searchParams) const {
    assert(vec);
    if (this->size(*this) == 0)
      return false;
    if (!BaseClassRef::root_node)
      return false;
    float epsError = 1 + searchParams.eps;

    distance_vector_t dists;

    assign(dists, (DIM > 0 ? DIM : BaseClassRef::dim),
           static_cast<typename distance_vector_t::value_type>(0));
    DistanceType distsq = this->computeInitialDistances(*this, vec, dists);
    searchLevel(result, vec, BaseClassRef::root_node, distsq, dists,
                epsError); 

    return result.full();
  }

  size_t knnSearch(const ElementType *query_point, const size_t num_closest,
                   IndexType *out_indices, DistanceType *out_distances_sq,
                   const int  = 10) const {
    nanoflann::KNNResultSet<DistanceType, IndexType> resultSet(num_closest);
    resultSet.init(out_indices, out_distances_sq);
    this->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
    return resultSet.size();
  }

  size_t
  radiusSearch(const ElementType *query_point, const DistanceType &radius,
               std::vector<std::pair<IndexType, DistanceType>> &IndicesDists,
               const SearchParams &searchParams) const {
    RadiusResultSet<DistanceType, IndexType> resultSet(radius, IndicesDists);
    const size_t nFound =
        radiusSearchCustomCallback(query_point, resultSet, searchParams);
    if (searchParams.sorted)
      std::sort(IndicesDists.begin(), IndicesDists.end(), IndexDist_Sorter());
    return nFound;
  }

  template <class SEARCH_CALLBACK>
  size_t radiusSearchCustomCallback(
      const ElementType *query_point, SEARCH_CALLBACK &resultSet,
      const SearchParams &searchParams = SearchParams()) const {
    this->findNeighbors(resultSet, query_point, searchParams);
    return resultSet.size();
  }

public:
  void computeBoundingBox(BoundingBox &bbox) {
    resize(bbox, (DIM > 0 ? DIM : BaseClassRef::dim));

    if (dataset.kdtree_get_bbox(bbox)) {

    } else {
      const size_t N = BaseClassRef::m_size;
      if (!N)
        throw std::runtime_error("[nanoflann] computeBoundingBox() called but "
                                 "no data points found.");
      for (int i = 0; i < (DIM > 0 ? DIM : BaseClassRef::dim); ++i) {
        bbox[i].low = bbox[i].high =
            this->dataset_get(*this, BaseClassRef::vind[0], i);
      }
      for (size_t k = 1; k < N; ++k) {
        for (int i = 0; i < (DIM > 0 ? DIM : BaseClassRef::dim); ++i) {
          if (this->dataset_get(*this, BaseClassRef::vind[k], i) < bbox[i].low)
            bbox[i].low = this->dataset_get(*this, BaseClassRef::vind[k], i);
          if (this->dataset_get(*this, BaseClassRef::vind[k], i) > bbox[i].high)
            bbox[i].high = this->dataset_get(*this, BaseClassRef::vind[k], i);
        }
      }
    }
  }

  template <class RESULTSET>
  void searchLevel(RESULTSET &result_set, const ElementType *vec,
                   const NodePtr node, DistanceType mindistsq,
                   distance_vector_t &dists, const float epsError) const {

    if ((node->child1 == NULL) && (node->child2 == NULL)) {

      DistanceType worst_dist = result_set.worstDist();
      for (IndexType i = node->node_type.lr.left; i < node->node_type.lr.right;
           ++i) {
        const IndexType index = BaseClassRef::vind[i]; 
        if (treeIndex[index] == -1)
          continue;
        DistanceType dist = distance.evalMetric(
            vec, index, (DIM > 0 ? DIM : BaseClassRef::dim));
        if (dist < worst_dist) {
          if (!result_set.addPoint(
                  static_cast<typename RESULTSET::DistanceType>(dist),
                  static_cast<typename RESULTSET::IndexType>(
                      BaseClassRef::vind[i]))) {

            return; 
          }
        }
      }
      return;
    }

    int idx = node->node_type.sub.divfeat;
    ElementType val = vec[idx];
    DistanceType diff1 = val - node->node_type.sub.divlow;
    DistanceType diff2 = val - node->node_type.sub.divhigh;

    NodePtr bestChild;
    NodePtr otherChild;
    DistanceType cut_dist;
    if ((diff1 + diff2) < 0) {
      bestChild = node->child1;
      otherChild = node->child2;
      cut_dist = distance.accum_dist(val, node->node_type.sub.divhigh, idx);
    } else {
      bestChild = node->child2;
      otherChild = node->child1;
      cut_dist = distance.accum_dist(val, node->node_type.sub.divlow, idx);
    }

    searchLevel(result_set, vec, bestChild, mindistsq, dists, epsError);

    DistanceType dst = dists[idx];
    mindistsq = mindistsq + cut_dist - dst;
    dists[idx] = cut_dist;
    if (mindistsq * epsError <= result_set.worstDist()) {
      searchLevel(result_set, vec, otherChild, mindistsq, dists, epsError);
    }
    dists[idx] = dst;
  }

public:

  void saveIndex(FILE *stream) { this->saveIndex_(*this, stream); }

  void loadIndex(FILE *stream) { this->loadIndex_(*this, stream); }
};

template <typename Distance, class DatasetAdaptor, int DIM = -1,
          typename IndexType = size_t>
class KDTreeSingleIndexDynamicAdaptor {
public:
  typedef typename Distance::ElementType ElementType;
  typedef typename Distance::DistanceType DistanceType;

protected:
  size_t m_leaf_max_size;
  size_t treeCount;
  size_t pointCount;

  const DatasetAdaptor &dataset; 

  std::vector<int> treeIndex; 

  KDTreeSingleIndexAdaptorParams index_params;

  int dim; 

  typedef KDTreeSingleIndexDynamicAdaptor_<Distance, DatasetAdaptor, DIM>
      index_container_t;
  std::vector<index_container_t> index;

public:

  const std::vector<index_container_t> &getAllIndices() const { return index; }

private:

  int First0Bit(IndexType num) {
    int pos = 0;
    while (num & 1) {
      num = num >> 1;
      pos++;
    }
    return pos;
  }

  void init() {
    typedef KDTreeSingleIndexDynamicAdaptor_<Distance, DatasetAdaptor, DIM>
        my_kd_tree_t;
    std::vector<my_kd_tree_t> index_(
        treeCount, my_kd_tree_t(dim , dataset, treeIndex, index_params));
    index = index_;
  }

public:
  Distance distance;

  KDTreeSingleIndexDynamicAdaptor(const int dimensionality,
                                  const DatasetAdaptor &inputData,
                                  const KDTreeSingleIndexAdaptorParams &params =
                                      KDTreeSingleIndexAdaptorParams(),
                                  const size_t maximumPointCount = 1000000000U)
      : dataset(inputData), index_params(params), distance(inputData) {
    treeCount = static_cast<size_t>(std::log2(maximumPointCount));
    pointCount = 0U;
    dim = dimensionality;
    treeIndex.clear();
    if (DIM > 0)
      dim = DIM;
    m_leaf_max_size = params.leaf_max_size;
    init();
    const size_t num_initial_points = dataset.kdtree_get_point_count();
    if (num_initial_points > 0) {
      addPoints(0, num_initial_points - 1);
    }
  }

  KDTreeSingleIndexDynamicAdaptor(
      const KDTreeSingleIndexDynamicAdaptor<Distance, DatasetAdaptor, DIM,
                                            IndexType> &) = delete;

  void addPoints(IndexType start, IndexType end) {
    size_t count = end - start + 1;
    treeIndex.resize(treeIndex.size() + count);
    for (IndexType idx = start; idx <= end; idx++) {
      int pos = First0Bit(pointCount);
      index[pos].vind.clear();
      treeIndex[pointCount] = pos;
      for (int i = 0; i < pos; i++) {
        for (int j = 0; j < static_cast<int>(index[i].vind.size()); j++) {
          index[pos].vind.push_back(index[i].vind[j]);
          if (treeIndex[index[i].vind[j]] != -1)
            treeIndex[index[i].vind[j]] = pos;
        }
        index[i].vind.clear();
        index[i].freeIndex(index[i]);
      }
      index[pos].vind.push_back(idx);
      index[pos].buildIndex();
      pointCount++;
    }
  }

  void removePoint(size_t idx) {
    if (idx >= pointCount)
      return;
    treeIndex[idx] = -1;
  }

  template <typename RESULTSET>
  bool findNeighbors(RESULTSET &result, const ElementType *vec,
                     const SearchParams &searchParams) const {
    for (size_t i = 0; i < treeCount; i++) {
      index[i].findNeighbors(result, &vec[0], searchParams);
    }
    return result.full();
  }
};

template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2>
struct KDTreeEigenMatrixAdaptor {
  typedef KDTreeEigenMatrixAdaptor<MatrixType, DIM, Distance> self_t;
  typedef typename MatrixType::Scalar num_t;
  typedef typename MatrixType::Index IndexType;
  typedef
      typename Distance::template traits<num_t, self_t>::distance_t metric_t;
  typedef KDTreeSingleIndexAdaptor<metric_t, self_t,
                                   MatrixType::ColsAtCompileTime, IndexType>
      index_t;

  index_t *index; 

  KDTreeEigenMatrixAdaptor(const size_t dimensionality,
                           const std::reference_wrapper<const MatrixType> &mat,
                           const int leaf_max_size = 10)
      : m_data_matrix(mat) {
    const auto dims = mat.get().cols();
    if (size_t(dims) != dimensionality)
      throw std::runtime_error(
          "Error: 'dimensionality' must match column count in data matrix");
    if (DIM > 0 && int(dims) != DIM)
      throw std::runtime_error(
          "Data set dimensionality does not match the 'DIM' template argument");
    index =
        new index_t(static_cast<int>(dims), *this ,
                    nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size));
    index->buildIndex();
  }

public:

  KDTreeEigenMatrixAdaptor(const self_t &) = delete;

  ~KDTreeEigenMatrixAdaptor() { delete index; }

  const std::reference_wrapper<const MatrixType> m_data_matrix;

  inline void query(const num_t *query_point, const size_t num_closest,
                    IndexType *out_indices, num_t *out_distances_sq,
                    const int  = 10) const {
    nanoflann::KNNResultSet<num_t, IndexType> resultSet(num_closest);
    resultSet.init(out_indices, out_distances_sq);
    index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
  }

  const self_t &derived() const { return *this; }
  self_t &derived() { return *this; }

  inline size_t kdtree_get_point_count() const {
    return m_data_matrix.get().rows();
  }

  inline num_t kdtree_get_pt(const IndexType idx, size_t dim) const {
    return m_data_matrix.get().coeff(idx, IndexType(dim));
  }

  template <class BBOX> bool kdtree_get_bbox(BBOX & ) const {
    return false;
  }

}; 

} 

#endif 
