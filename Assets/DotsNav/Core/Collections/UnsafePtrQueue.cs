using System;


namespace Unity.Collections.LowLevel.Unsafe {
unsafe struct UnsafePtrQueue<T> : IDisposable
    where T : unmanaged
{
    UnsafePtrList<T> data;
    public int Length { get; private set; }
    public int Capacity => data.Capacity;

    int front;
    int rear;

    public UnsafePtrQueue(int initialCapacity, Allocator allocator)
    {
        data = new UnsafePtrList<T>(initialCapacity, allocator);
        front = rear = -1;
        Length = 0;
    }

    public void Enqueue(T* elementPtr) {
        if ((front == 0 && rear == Capacity - 1) || (rear == (front - 1))) {
            // UnityEngine.Debug.Log("Hehe");
            data.Add(null);
            front = 0;
            rear = Length;
            data[rear] = elementPtr;
        } else {
            if (front == -1 && rear == -1) { // First element added so set front and rear
                front = rear = 0;
            } else if (rear == Capacity - 1) { // Tail reached end so wrap it back to beginning
                // UnityEngine.Debug.Log("rear wrapped");
                rear = 0;
            } else { // Normal increment
                rear++;
            }
            data.Add(null);
            data[rear] = elementPtr;
            Length++;
        }
        // UnityEngine.Debug.Log("Capacity: " + Capacity + ",   " + "Length: " + data.Length);
    }
    
    public T* Dequeue() {
        if (IsEmpty) {
            return null;
        } else {
            T* elementPtr = data[front];
            data[front] = null;
            if (front == rear) { // Final item in queue so set back to empty
                front = rear = -1;
            } else if (front == Capacity - 1) { // If front is at end then wrap it back to beginning
                // UnityEngine.Debug.Log("front wrapped");
                front = 0;
            } else { // Normal increment
                front++;
            }
            Length--;
            return elementPtr;
        }
    }

    public T* Peek() {
        if (IsEmpty) {
            return null;
        } else {
            return data[front];
        }
    }

    public void Print() {
        if (IsEmpty) {
            return;
        }
        UnityEngine.Debug.Log("Capacity: " + Capacity + ",   " + "front: " + front + ",   " + "rear: " + rear);
        for (int i = 0; i < data.Length; i++) {
            if (data[i] == null) {
                UnityEngine.Debug.Log("null");
            } else {
                UnityEngine.Debug.Log("i: " + i + " == " + *data[i]);
            }
        }
    }

    public bool IsEmpty => front == -1 && rear == -1;

    public T* this[int i] => data[i];

    public void Clear() {
        data.Clear();
        front = rear = -1;
    }

    public void Dispose() => data.Dispose();
}
}