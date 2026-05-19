# mullet_core Interactive Docs

This directory contains a self-contained Vite React + TypeScript docs app for
`mullet_core`. It is intentionally separate from the ROS 2/CMake build.

## Commands

```bash
npm install
npm run dev
npm run typecheck
npm run build
```

The generated `dist/` directory and `node_modules/` are ignored. Documentation
content lives in `src/data/mulletCoreModel.ts`; update that model in the same PR
as state-machine behavior changes so the app stays aligned with the C++ code.
