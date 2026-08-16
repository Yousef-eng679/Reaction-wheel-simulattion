import { defineConfig } from 'vite';

export default defineConfig({
  // Vitest configuration (co-located in vite.config.ts per Vitest best practice)
  test: {
    // Run tests in Node environment (no browser needed for physics/control tests)
    environment: 'node',
    // Include all test files in src/tests/
    include: ['src/tests/**/*.test.ts'],
    // Globals: use explicit import { describe, it, expect } from 'vitest'
    globals: false,
  },
});
