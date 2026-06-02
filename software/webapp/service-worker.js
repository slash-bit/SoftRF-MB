/**
 * Service Worker for SoftRF Settings
 * Enables offline support and caching of assets.
 * Cache name is updated by the page via SET_VERSION message so that
 * bumping APP_VERSION in index.html automatically evicts stale cached assets.
 */

// Bump this string whenever you want all clients to get fresh assets.
// The browser always re-fetches service-worker.js, so a change here
// triggers install → activate → old cache deleted → fresh index.html fetched.
let CACHE_NAME = 'softrf-v1.7';

const ASSETS_TO_CACHE = [
  './',
  './index.html',
  './icon-192.png',
  './icon-512.png',
  './icon-maskable-512.png',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
  'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png',
  'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
  'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon-2x.png'
];

const OFFLINE_HTML = `<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SoftRF - Offline</title><style>body{font-family:sans-serif;background:#0f0f1a;color:#e5e7eb;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:100vh;margin:0;text-align:center}.logo{width:80px;height:80px;background:#4361ee;border-radius:16px;display:flex;align-items:center;justify-content:center;font-size:36px;font-weight:bold;color:#fff;margin-bottom:24px}h1{font-size:24px;margin-bottom:8px}p{color:#9ca3af;max-width:300px}button{margin-top:24px;padding:12px 24px;background:#4361ee;color:#fff;border:none;border-radius:8px;font-size:16px;cursor:pointer}</style></head><body><div class="logo">SR</div><h1>You are offline</h1><p>SoftRF Configurator requires a connection to load. Please connect to the internet or your device's Wi-Fi hotspot.</p><button onclick="location.reload()">Try Again</button></body></html>`;

/**
 * Install event - cache essential assets
 */
self.addEventListener('install', event => {
  console.log('[Service Worker] Installing...');
  event.waitUntil(
    caches.open(CACHE_NAME).then(cache => {
      console.log('[Service Worker] Caching assets');
      return cache.addAll(ASSETS_TO_CACHE).catch(err => {
        console.warn('[Service Worker] Some assets failed to cache:', err);
        return Promise.resolve();
      });
    }).then(() => self.skipWaiting())
  );
});

/**
 * Activate event - clean up old caches
 */
self.addEventListener('activate', event => {
  console.log('[Service Worker] Activating...');
  event.waitUntil(
    caches.keys().then(cacheNames => {
      return Promise.all(
        cacheNames.map(cacheName => {
          if (cacheName !== CACHE_NAME) {
            console.log('[Service Worker] Deleting old cache:', cacheName);
            return caches.delete(cacheName);
          }
        })
      );
    }).then(() => self.clients.claim())
  );
});


/**
 * Fetch event - serve from cache, fallback to network
 * Strategy: Cache first for same-origin; network first for CDN.
 */
self.addEventListener('fetch', event => {
  const { request } = event;
  const url = new URL(request.url);

  // Skip non-GET requests
  if (request.method !== 'GET') {
    return;
  }

  // For index.html: network-first so updates are picked up automatically.
  // For other same-origin assets: cache-first for performance.
  if (url.origin === location.origin) {
    const isHtml = url.pathname.endsWith('/') || url.pathname.endsWith('.html');
    if (isHtml) {
      event.respondWith(
        fetch(request).then(response => {
          if (response && response.status === 200) {
            const responseToCache = response.clone();
            caches.open(CACHE_NAME).then(cache => cache.put(request, responseToCache));
          }
          return response;
        }).catch(() => caches.match(request).then(response =>
          response || new Response(OFFLINE_HTML, {
            status: 503,
            statusText: 'Service Unavailable',
            headers: new Headers({ 'Content-Type': 'text/html; charset=utf-8' })
          })
        ))
      );
    } else {
      event.respondWith(
        caches.match(request).then(response => {
          if (response) return response;
          return fetch(request).then(response => {
            if (response && response.status === 200) {
              const responseToCache = response.clone();
              caches.open(CACHE_NAME).then(cache => cache.put(request, responseToCache));
            }
            return response;
          }).catch(() => new Response(OFFLINE_HTML, {
            status: 503,
            statusText: 'Service Unavailable',
            headers: new Headers({ 'Content-Type': 'text/html; charset=utf-8' })
          }));
        })
      );
    }
    return;
  }

  // For CDN/external requests (Leaflet), use network-first strategy with cache fallback
  if (url.hostname.includes('unpkg.com')) {
    event.respondWith(
      fetch(request)
        .then(response => {
          if (response && response.status === 200) {
            const responseToCache = response.clone();
            caches.open(CACHE_NAME).then(cache => {
              cache.put(request, responseToCache);
            });
          }
          return response;
        })
        .catch(() => {
          return caches.match(request).then(response => {
            if (response) {
              return response;
            }
            return new Response('Offline - resource not cached', {
              status: 503,
              statusText: 'Service Unavailable',
              headers: new Headers({ 'Content-Type': 'text/plain' })
            });
          });
        })
    );
    return;
  }
});

/**
 * Message handler — allows the page to pass APP_VERSION so the cache
 * key stays in sync without editing this file directly.
 */
self.addEventListener('message', event => {
  if (event.data && event.data.type === 'SET_VERSION') {
    const newCache = 'softrf-' + event.data.version;
    if (newCache !== CACHE_NAME) {
      CACHE_NAME = newCache;
    }
  }
  if (event.data && event.data.type === 'SKIP_WAITING') {
    self.skipWaiting();
  }
});

/**
 * Background sync for sending data when connection is restored
 */
self.addEventListener('sync', event => {
  if (event.tag === 'sync-data') {
    event.waitUntil(syncData());
  }
});

async function syncData() {
  console.log('[Service Worker] Background sync triggered');
}

/**
 * Handle push notifications
 */
self.addEventListener('push', event => {
  if (event.data) {
    const options = {
      body: event.data.text(),
      icon: 'data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 192 192"><rect fill="%234361ee" width="192" height="192"/><text x="50%" y="50%" font-size="120" font-weight="bold" fill="white" text-anchor="middle" dominant-baseline="central">SR</text></svg>',
      badge: 'data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 192 192"><rect fill="%234361ee" width="192" height="192"/><text x="50%" y="50%" font-size="120" font-weight="bold" fill="white" text-anchor="middle" dominant-baseline="central">SR</text></svg>'
    };
    self.registration.showNotification('SoftRF', options);
  }
});

console.log('[Service Worker] Loaded');
