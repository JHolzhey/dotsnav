using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Debug = UnityEngine.Debug;

namespace DotsNav.Navmesh
{
    public unsafe partial struct Navmesh
    {
        /// <summary>
        /// Enumerates edges in the triangulation. Updating the navmesh invalidates this structure.
        /// </summary>
        public struct EdgeEnumerator
        {
            UnsafeList<IntPtr> _vertices;
            readonly bool _sym;
            readonly bool _isMajor;

            int _vertexIndex;
            bool _started;
            Vertex.EdgeEnumerator _enumerator;

            /// <summary>
            /// Current edge being enumerated.
            /// </summary>
            public Edge* Current => _enumerator.Current;
            public Vertex* CurrentVertex => (Vertex*) (_vertexIndex >= _vertices.Length ? _vertices[_vertexIndex - 1] : _vertices[_vertexIndex]);

            readonly float2 _max;

            internal EdgeEnumerator(UnsafeList<IntPtr> vertices, float2 max, bool isMajor, bool sym) : this()
            {
                _vertices = vertices;
                _max = max;
                _isMajor = isMajor;
                _sym = sym;
            }

            /// <summary>
            /// Assigns the next edge to Current and returns true while the enumeration continues, returns false otherwise
            /// </summary>
            public bool MoveNext()
            {
                if (!_started) 
                {
                    if (_vertices.Length == 0)
                        return false;

                    _enumerator = ((Vertex*) _vertices[_vertexIndex++])->GetEdgeEnumerator(_isMajor);
                    _started = true;
                }

                do
                {
                    while (!_enumerator.MoveNext())
                    {
                        if (_vertexIndex == _vertices.Length)
                            return false;

                        _enumerator = ((Vertex*) _vertices[_vertexIndex++])->GetEdgeEnumerator(_isMajor);
                    }

                } while
                (
                    Current == null || math.any(math.max(math.abs(Current->Org->Point), math.abs(Current->Dest->Point)) > _max) ||
                    !_sym && !Current->IsPrimary
                );

                return true;
            }
        }
    }
}