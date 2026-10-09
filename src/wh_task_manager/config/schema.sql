-- schema.sql – CSDL Quản lý Tồn kho & Nhiệm vụ Kho hàng (SQLite)

CREATE TABLE IF NOT EXISTS items(
    item_code TEXT PRIMARY KEY,
    name TEXT,
    box_tag_id INTEGER UNIQUE,
    low_stock_threshold INTEGER DEFAULT 1
);

CREATE TABLE IF NOT EXISTS slots(
    slot_id TEXT PRIMARY KEY,
    rack TEXT,
    col INTEGER,
    tier INTEGER,
    slot_tag_id INTEGER UNIQUE,
    item_code TEXT REFERENCES items(item_code),
    quantity INTEGER DEFAULT 0,
    state TEXT DEFAULT 'EMPTY',   -- EMPTY | FULL | RESERVED
    reserved_by_task INTEGER,
    stored_at TEXT
);

CREATE TABLE IF NOT EXISTS tasks(
    task_id INTEGER PRIMARY KEY AUTOINCREMENT,
    type TEXT,
    item_code TEXT,
    slot_id TEXT,
    priority INTEGER DEFAULT 1,
    status TEXT DEFAULT 'QUEUED', -- QUEUED | RUNNING | SUCCEEDED | FAILED | CANCELLED
    created_at TEXT,
    started_at TEXT,
    finished_at TEXT,
    error TEXT
);

CREATE TABLE IF NOT EXISTS transactions(
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id INTEGER,
    type TEXT,
    item_code TEXT,
    slot_id TEXT,
    delta INTEGER,
    ts TEXT
);

CREATE TABLE IF NOT EXISTS events(
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    level TEXT,
    message TEXT,
    ts TEXT
);
