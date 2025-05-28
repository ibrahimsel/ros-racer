####################  build phase  ####################
FROM node:20-alpine AS builder

WORKDIR /app
COPY . .

RUN npm ci && npm run build       # outputs to /app/dist

####################  run phase  ####################
FROM nginx:1.27-alpine
COPY --from=builder /app/build /usr/share/nginx/html
EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
