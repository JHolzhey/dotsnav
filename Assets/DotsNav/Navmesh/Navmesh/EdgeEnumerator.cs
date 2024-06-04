using System.Diagnostics;
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

            int _vertexIndex;
            bool _started;
            Vertex.EdgeEnumerator _enumerator;

            /// <summary>
            /// Current edge being enumerated.
            /// </summary>
            public Edge* Current => _enumerator.Current;
            public Vertex* CurrentVertex { 
                get {
                    if (_vertexIndex >= _vertices.Length) {
                        return (Vertex*) _vertices[_vertexIndex - 1];
                    } else {
                        return (Vertex*) _vertices[_vertexIndex];
                    }
                }
            }

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
                    while (!_enumerator.MoveNext()) // || _enumerator.Current == null) // TODO: Remove second condition?
                    {
                        if (_vertexIndex == _vertices.Length)
                            return false;

                        _enumerator = ((Vertex*) _vertices[_vertexIndex++])->GetEdgeEnumerator(_isMajor);
                    }

                    // TODO: Debug, delete
                    if (Current == null) {
                        UnityEngine.Debug.LogError("Current is null");
                        break;
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