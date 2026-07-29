from pathlib import Path
import numpy as np

_TYPES = {1: ('i1', 1), 2: ('u1', 1), 3: ('i2', 2), 4: ('u2', 2), 5: ('i4', 4), 6: ('u4', 4), 7: ('f4', 4), 8: ('f8', 8)}

def pointcloud_xyzi(msg):
    fields = {field.name: field for field in msg.fields}
    missing = {'x', 'y', 'z'} - set(fields)
    if missing: raise ValueError(f'PointCloud2 missing fields: {sorted(missing)}')
    endian = '>' if getattr(msg, 'is_bigendian', False) else '<'
    dtype = np.dtype({'names': list(fields), 'formats': [endian + _TYPES[field.datatype][0] for field in fields.values()], 'offsets': [field.offset for field in fields.values()], 'itemsize': msg.point_step})
    records = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    intensity = records['intensity'] if 'intensity' in records.dtype.names else np.zeros(records.shape, dtype=np.float32)
    result = np.column_stack((records['x'], records['y'], records['z'], intensity)).astype(np.float32, copy=False)
    return result[np.isfinite(result[:, :3]).all(axis=1)]

def write_pcd(path, points, binary=True):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points, dtype=np.float32).reshape((-1, 4))
    mode = 'binary' if binary else 'ascii'
    header = f'''# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA {mode}\n'''
    with path.open('wb') as handle:
        handle.write(header.encode('ascii'))
        if binary: handle.write(points.astype('<f4', copy=False).tobytes())
        else: np.savetxt(handle, points, fmt='%.8f')
