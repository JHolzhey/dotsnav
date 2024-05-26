using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

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

            int _vertex;
            bool _started;
            Vertex.EdgeEnumerator _enumerator;

            /// <summary>
            /// Current edge being enumerated.
            /// </summary>
            public Edge* Current => _enumerator.Current;

            readonly float2 _max;

            internal EdgeEnumerator(UnsafeList<IntPtr> vertices, float2 max, bool sym, bool isMajor) : this()
            {
                _vertices = vertices;
                _sym = sym;
                _isMajor = isMajor;
                _max = max;
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

                    _enumerator = ((Vertex*) _vertices[_vertex++])->GetEdgeEnumerator(_isMajor);
                    _started = true;
                }

                do
                {
                    while (!_enumerator.MoveNext())
                    {
                        if (_vertex == _vertices.Length)
                            return false;

                        _enumerator = ((Vertex*) _vertices[_vertex++])->GetEdgeEnumerator(_isMajor);
                    }


                } while
                (
                    math.any(math.max(math.abs(Current->Org->Point), math.abs(Current->Dest->Point)) > _max) ||
                    !_sym && !Current->IsPrimary
                );

                return true;
            }
        }
    }
}