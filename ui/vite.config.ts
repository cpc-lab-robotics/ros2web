import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vitejs.dev/config/
export default defineConfig({
  resolve: {
    alias: [{ find: '@', replacement: '/src' }]
  },
  plugins: [react()],
  server: {
    proxy: {
      '^/subscription': {
        target: 'ws://localhost:8080',
        changeOrigin: true,
      },
      '^/api': {
        target: 'http://localhost:8080',
        changeOrigin: true,
      }
      
    }
  },
})
