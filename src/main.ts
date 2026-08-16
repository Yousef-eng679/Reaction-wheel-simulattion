/**
 * src/main.ts — root entry point (re-exports ui/main.ts)
 *
 * This file is the Vite entry point (referenced in index.html).
 * All actual UI wiring lives in src/ui/main.ts, per the folder structure
 * defined in IMPLEMENTATION_PLAN.md Phase 0.
 *
 * In Phase 5, this file will import and call the full UI setup from ui/main.ts.
 * For Phase 0, it just imports the placeholder page from ui/main.ts.
 */

import './ui/main.ts';
