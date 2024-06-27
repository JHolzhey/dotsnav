using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

public unsafe struct Triangulator {
    Heightmap m_Heightmap;

    UnsafeList<int2> m_Points;

    UnsafeList<int> m_Triangles;
    UnsafeList<int> m_Halfedges;

    UnsafeList<int2> m_Candidates;
    UnsafeList<float> m_Errors;
    UnsafeList<int> m_QueueIndexes;

    UnsafeList<int> m_Queue;

    UnsafeList<int> m_Pending;

    public Triangulator(Heightmap heightmap) {
        m_Heightmap = heightmap;
        m_Points = new UnsafeList<int2>(1, Allocator.Temp);
        m_Triangles = new UnsafeList<int>(1, Allocator.Temp);
        m_Halfedges = new UnsafeList<int>(1, Allocator.Temp);
        m_Candidates = new UnsafeList<int2>(1, Allocator.Temp);
        m_Errors = new UnsafeList<float>(1, Allocator.Temp);
        m_QueueIndexes = new UnsafeList<int>(1, Allocator.Temp);
        m_Queue = new UnsafeList<int>(1, Allocator.Temp);
        m_Pending = new UnsafeList<int>(1, Allocator.Temp);
    }

    public void Run(float maxError, int maxTriangles, int maxPoints)
    {
        // add points at all four corners
        int x0 = 0;
        int y0 = 0;
        int x1 = m_Heightmap.Width() - 1;
        int y1 = m_Heightmap.Length() - 1;
        int p0 = AddPoint(new int2(x0, y0));
        int p1 = AddPoint(new int2(x1, y0));
        int p2 = AddPoint(new int2(x0, y1));
        int p3 = AddPoint(new int2(x1, y1));

        // add initial two triangles
        int t0 = AddTriangle(p3, p0, p2, -1, -1, -1, -1);
        AddTriangle(p0, p3, p1, t0, -1, -1, -1);
        Flush();

        while (!done(maxError, maxTriangles, maxPoints)) {
            Step();
        }
    }

    // helper function to check if triangulation is complete
    bool done(float maxError, int maxTriangles, int maxPoints) {
        float e = Error();
        if (e <= maxError) {
            return true;
        }
        if (maxTriangles > 0 && NumTriangles() >= maxTriangles) {
            return true;
        }
        if (maxPoints > 0 && NumPoints() >= maxPoints) {
            return true;
        }
        return e == 0;
    }

    public int NumPoints() {
        return m_Points.Length;
    }

    public int NumTriangles() {
        return m_Queue.Length;
    }

    
    public float Error() {
        return m_Errors[m_Queue[0]];
    }

    public UnsafeList<float3> Points(float heightScaleFactor = 1f) {
        UnsafeList<float3> points = new (m_Points.Length, Allocator.Temp);
        // int l1 = m_Heightmap.Length() - 1; // This was originally added to -p.y below, not sure why
        foreach (int2 p in m_Points) {
            points.Add(new float3(p.x, m_Heightmap.At(p.x, p.y) * heightScaleFactor, p.y));
        }
        return points;
    }

    public UnsafeList<int3> Triangles() {
        UnsafeList<int3> triangles = new (m_Queue.Length, Allocator.Temp);
        foreach (int i in m_Queue) {
            triangles.Add(new int3(
                m_Triangles[i * 3 + 0],
                m_Triangles[i * 3 + 1],
                m_Triangles[i * 3 + 2]));
        }
        return triangles;
    }

    
    void Flush() {
        foreach (int t in m_Pending) {
            // rasterize triangle to find maximum pixel error
            Pair<int2, float> pair = m_Heightmap.FindCandidate(
                m_Points[m_Triangles[t*3+0]],
                m_Points[m_Triangles[t*3+1]],
                m_Points[m_Triangles[t*3+2]]);
            // update metadata
            m_Candidates[t] = pair.first;
            m_Errors[t] = pair.second;
            // add triangle to priority queue
            QueuePush(t);
        }

        m_Pending.Clear();
    }

    void Step() {
        // pop triangle with highest error from priority queue
        int t = QueuePop();

        int e0 = t * 3 + 0;
        int e1 = t * 3 + 1;
        int e2 = t * 3 + 2;

        int p0 = m_Triangles[e0];
        int p1 = m_Triangles[e1];
        int p2 = m_Triangles[e2];

        int2 a = m_Points[p0];
        int2 b = m_Points[p1];
        int2 c = m_Points[p2];
        int2 p = m_Candidates[t];

        int pn = AddPoint(p);

        bool collinear(int2 p0, int2 p1, int2 p2)
        {
            return (p1.y-p0.y)*(p2.x-p1.x) == (p2.y-p1.y)*(p1.x-p0.x);
        };

        if (collinear(a, b, p)) {
            handleCollinear(pn, e0);
        } else if (collinear(b, c, p)) {
            handleCollinear(pn, e1);
        } else if (collinear(c, a, p)) {
            handleCollinear(pn, e2);
        } else {
            int h0 = m_Halfedges[e0];
            int h1 = m_Halfedges[e1];
            int h2 = m_Halfedges[e2];

            int t0 = AddTriangle(p0, p1, pn, h0, -1, -1, e0);
            int t1 = AddTriangle(p1, p2, pn, h1, -1, t0 + 1, -1);
            int t2 = AddTriangle(p2, p0, pn, h2, t0 + 2, t1 + 1, -1);

            Legalize(t0);
            Legalize(t1);
            Legalize(t2);
        }

        Flush();
    }

    void handleCollinear(int pn, int a) {
        int a0 = a - a % 3;
        int al = a0 + (a + 1) % 3;
        int ar = a0 + (a + 2) % 3;
        int p0 = m_Triangles[ar];
        int pr = m_Triangles[a];
        int pl = m_Triangles[al];
        int hal = m_Halfedges[al];
        int har = m_Halfedges[ar];

        int b = m_Halfedges[a];

        if (b < 0) {
            int t0_ = AddTriangle(pn, p0, pr, -1, har, -1, a0);
            int t1_ = AddTriangle(p0, pn, pl, t0_, -1, hal, -1);
            Legalize(t0_ + 1);
            Legalize(t1_ + 2);
            return;
        }

        int b0 = b - b % 3;
        int bl = b0 + (b + 2) % 3;
        int br = b0 + (b + 1) % 3;
        int p1 = m_Triangles[bl];
        int hbl = m_Halfedges[bl];
        int hbr = m_Halfedges[br];

        QueueRemove(b / 3);

        int t0 = AddTriangle(p0, pr, pn, har, -1, -1, a0);
        int t1 = AddTriangle(pr, p1, pn, hbr, -1, t0 + 1, b0);
        int t2 = AddTriangle(p1, pl, pn, hbl, -1, t1 + 1, -1);
        int t3 = AddTriangle(pl, p0, pn, hal, t0 + 2, t2 + 1, -1);

        Legalize(t0);
        Legalize(t1);
        Legalize(t2);
        Legalize(t3);
    }

    int AddPoint(int2 point) {
        int i = m_Points.Length;
        m_Points.Add(point);
        return i;
    }

    int AddTriangle(
        int a, int b, int c,
        int ab, int bc, int ca,
        int e)
    {
        if (e < 0) {
            // new halfedge index
            e = m_Triangles.Length;
            // add triangle vertices
            m_Triangles.Add(a);
            m_Triangles.Add(b);
            m_Triangles.Add(c);
            // add triangle halfedges
            m_Halfedges.Add(ab);
            m_Halfedges.Add(bc);
            m_Halfedges.Add(ca);
            // add triangle metadata
            m_Candidates.Add(new int2(0));
            m_Errors.Add(0);
            m_QueueIndexes.Add(-1);
        } else {
            // set triangle vertices
            m_Triangles[e + 0] = a;
            m_Triangles[e + 1] = b;
            m_Triangles[e + 2] = c;
            // set triangle halfedges
            m_Halfedges[e + 0] = ab;
            m_Halfedges[e + 1] = bc;
            m_Halfedges[e + 2] = ca;
        }

        // link neighboring halfedges
        if (ab >= 0) {
            m_Halfedges[ab] = e + 0;
        }
        if (bc >= 0) {
            m_Halfedges[bc] = e + 1;
        }
        if (ca >= 0) {
            m_Halfedges[ca] = e + 2;
        }

        // add triangle to pending queue for later rasterization
        int t = e / 3;
        m_Pending.Add(t);

        // return first halfedge index
        return e;
    }

    void Legalize(int a) {
        // if the pair of triangles doesn't satisfy the Delaunay condition
        // (p1 is inside the circumcircle of [p0, pl, pr]), flip them,
        // then do the same check/flip recursively for the new pair of triangles
        //
        //           pl                    pl
        //          /||\                  /  \
        //       al/ || \bl            al/    \a
        //        /  ||  \              /      \
        //       /  a||b  \    flip    /___ar___\
        //     p0\   ||   /p1   =>   p0\---bl---/p1
        //        \  ||  /              \      /
        //       ar\ || /br             b\    /br
        //          \||/                  \  /
        //           pr                    pr

        bool inCircle(
            int2 a, int2 b, int2 c,
            int2 p)
        {
            long dx = a.x - p.x;
            long dy = a.y - p.y;
            long ex = b.x - p.x;
            long ey = b.y - p.y;
            long fx = c.x - p.x;
            long fy = c.y - p.y;
            long ap = dx * dx + dy * dy;
            long bp = ex * ex + ey * ey;
            long cp = fx * fx + fy * fy;
            return dx*(ey*cp-bp*fy)-dy*(ex*cp-bp*fx)+ap*(ex*fy-ey*fx) < 0;
        };

        int b = m_Halfedges[a];

        if (b < 0) {
            return;
        }

        int a0 = a - a % 3;
        int b0 = b - b % 3;
        int al = a0 + (a + 1) % 3;
        int ar = a0 + (a + 2) % 3;
        int bl = b0 + (b + 2) % 3;
        int br = b0 + (b + 1) % 3;
        int p0 = m_Triangles[ar];
        int pr = m_Triangles[a];
        int pl = m_Triangles[al];
        int p1 = m_Triangles[bl];

        if (!inCircle(m_Points[p0], m_Points[pr], m_Points[pl], m_Points[p1])) {
            return;
        }

        int hal = m_Halfedges[al];
        int har = m_Halfedges[ar];
        int hbl = m_Halfedges[bl];
        int hbr = m_Halfedges[br];

        QueueRemove(a / 3);
        QueueRemove(b / 3);

        int t0 = AddTriangle(p0, p1, pl, -1, hbl, hal, a0);
        int t1 = AddTriangle(p1, p0, pr, t0, har, hbr, b0);

        Legalize(t0 + 1);
        Legalize(t1 + 2);
    }

    // priority queue functions

    void QueuePush(int t) {
        int i = m_Queue.Length;
        m_QueueIndexes[t] = i;
        m_Queue.Add(t);
        QueueUp(i);
    }

    int QueuePop() {
        int n = m_Queue.Length - 1;
        QueueSwap(0, n);
        QueueDown(0, n);
        return QueuePopBack();
    }

    int QueuePopBack() {
        int t = m_Queue[m_Queue.Length - 1]; //.back();
        m_Queue.RemoveAt(m_Queue.Length - 1);
        m_QueueIndexes[t] = -1;
        return t;
    }

    void QueueRemove(int t) {
        int i = m_QueueIndexes[t];
        if (i < 0) {
            int indexOf = m_Pending.IndexOf(t);
            if (indexOf != -1) {
                m_Pending.RemoveAtSwapBack(indexOf);
            } else {
                UnityEngine.Debug.Assert(false, "this shouldn't happen!");
            }
            // auto it = std::find(m_Pending.begin(), m_Pending.end(), t);
            // if (it != m_Pending.end()) {
            //     std::swap(*it, m_Pending.back());
            //     m_Pending.pop_back();
            // } else {
            //     // this shouldn't happen!
            // }
            return;
        }
        int n = m_Queue.Length - 1;
        if (n != i) {
            QueueSwap(i, n);
            if (!QueueDown(i, n)) {
                QueueUp(i);
            }
        }
        QueuePopBack();
    }

    bool QueueLess(int i, int j) {
        return -m_Errors[m_Queue[i]] < -m_Errors[m_Queue[j]];
    }

    void QueueSwap(int i, int j) {
        int pi = m_Queue[i];
        int pj = m_Queue[j];
        m_Queue[i] = pj;
        m_Queue[j] = pi;
        m_QueueIndexes[pi] = j;
        m_QueueIndexes[pj] = i;
    }

    void QueueUp(int j0) {
        int j = j0;
        while (true) {
            int i = (j - 1) / 2;
            if (i == j || !QueueLess(j, i)) {
                break;
            }
            QueueSwap(i, j);
            j = i;
        }
    }

    bool QueueDown(int i0, int n) {
        int i = i0;
        while (true) {
            int j1 = 2 * i + 1;
            if (j1 >= n || j1 < 0) {
                break;
            }
            int j2 = j1 + 1;
            int j = j1;
            if (j2 < n && QueueLess(j2, j1)) {
                j = j2;
            }
            if (!QueueLess(j, i)) {
                break;
            }
            QueueSwap(i, j);
            i = j;
        }
        return i > i0;
    }
};
