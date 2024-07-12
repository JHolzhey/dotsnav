using DotsNav.Drawing;
using Unity.Entities;

namespace DotsNav.Systems
{
    [UpdateInGroup(typeof(DotsNavDrawingSystemGroup), OrderLast = true)]
    partial class ClearSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            // DotsNavRenderer.Clear();
        }
    }
}